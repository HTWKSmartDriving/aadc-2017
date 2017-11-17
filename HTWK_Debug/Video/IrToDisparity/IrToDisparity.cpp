#include "IrToDisparity.h"

ADTF_FILTER_PLUGIN(OID_ADTF_FILTER_DEF, OID_ADTF_FILTER_DEF, IrToDisparity);

IrToDisparity::IrToDisparity(const tChar* __info) : cFilter(__info) {
    SetPropertyInt(PROPERTY_MAX_DISP, MAXIMUM_DISPARITY_DEFAULT);
    SetPropertyStr(PROPERTY_MAX_DISP NSSUBPROP_DESCRIPTION,
                   "Maximum disparity, must be dividable 16");
    SetPropertyInt(PROPERTY_MAX_DISP NSSUBPROP_MIN, 16);
    SetPropertyInt(PROPERTY_MAX_DISP NSSUBPROP_MAX, 4096);
}

IrToDisparity::~IrToDisparity() {}

tResult IrToDisparity::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst) {
        cObjectPtr<IMediaType> pOutputType;
        RETURN_IF_FAILED(AllocMediaType(&pOutputType,
                                        MEDIA_TYPE_CAMERA_PROPERTY,
                                        MEDIA_SUBTYPE_CAMERA_PROPERTY,
                                        __exception_ptr));
        outputPinDisparity.Create("Disparity", IPin::PD_Output, this);
        RegisterPin(&outputPinDisparity);
        irLeftInputPin.Create("ir_left", IPin::PD_Input, this);
        RegisterPin(&irLeftInputPin);
        irRightInputPin.Create("ir_right", IPin::PD_Input, this);
        RegisterPin(&irRightInputPin);
    } else if (eStage == StageNormal) {
        bitmapFormatDisparity.nWidth = WIDTH;
        bitmapFormatDisparity.nHeight = HEIGHT;
        bitmapFormatDisparity.nBitsPerPixel = 32;
        bitmapFormatDisparity.nPixelFormat = IImage::PF_GREYSCALE_FLOAT32;
        bitmapFormatDisparity.nBytesPerLine =
                bitmapFormatDisparity.nWidth * bitmapFormatDisparity.nBitsPerPixel / 8;
        bitmapFormatDisparity.nSize =
                bitmapFormatDisparity.nBytesPerLine * bitmapFormatDisparity.nHeight;
        bitmapFormatDisparity.nPaletteSize = 0;
        outputPinDisparity.SetFormat(&bitmapFormatDisparity, NULL);
    } else if (eStage == StageGraphReady) {
        {
            cObjectPtr<IMediaType> pType;
            RETURN_IF_FAILED(irLeftInputPin.GetMediaType(&pType));
            cObjectPtr<IMediaTypeVideo> pTypeVideo;
            RETURN_IF_FAILED(
                    pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**) &pTypeVideo));
            updateInputImageFormat(pTypeVideo->GetFormat(), irLeftInputBitmapFormat,
                                   irLeftInputImage);
        }
        {
            cObjectPtr<IMediaType> pType;
            RETURN_IF_FAILED(irRightInputPin.GetMediaType(&pType));
            cObjectPtr<IMediaTypeVideo> pTypeVideo;
            RETURN_IF_FAILED(
                    pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**) &pTypeVideo));
            updateInputImageFormat(pTypeVideo->GetFormat(), irRightInputBitmapFormat,
                                   irRightInputImage);
        }
    }
    RETURN_NOERROR;
}

tResult IrToDisparity::Start(__exception) {
    maxDisparity = GetPropertyInt(PROPERTY_MAX_DISP);
    if (maxDisparity % FACTOR_16 != 0) { //SGBM needs maxDisparity dividable 16
        LOG_WARNING("Maximum Disparity must be dividable 16! Standard value is now used");
        maxDisparity = MAXIMUM_DISPARITY_DEFAULT;
    }
    initSGBM();
    return cFilter::Start(__exception_ptr);
}

tResult IrToDisparity::Stop(__exception) {
    return cFilter::Stop(__exception_ptr);
}

tResult IrToDisparity::Shutdown(tInitStage eStage, __exception) {
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult IrToDisparity::Process(IMediaSample* pData, IPin* pSource) {
    const tTimeStamp time = pData->GetTime();
    bool newIrImage = false;
    if (pSource == &irLeftInputPin) {
        if (time != timeLeft) {
            timeLeft = time;
            RETURN_IF_FAILED(bufferToMat(irLeftInputPin.GetFormat(), pData,
                                         irLeftInputImage, irLeftInputBitmapFormat));
            newIrImage = true;
        }
    } else if (pSource == &irRightInputPin) {
        if (time != timeRight) {
            timeRight = time;
            RETURN_IF_FAILED(bufferToMat(irRightInputPin.GetFormat(), pData,
                                         irRightInputImage, irRightInputBitmapFormat));
            newIrImage = true;
        }
    }
    if (newIrImage && timeLeft == timeRight && outputPinDisparity.IsConnected()) {
        cv::Mat disparityMap;
        stereoSGBM->compute(irLeftInputImage, irRightInputImage, disparityMap); //CV_16S
        disparityMap.convertTo(disparityMap, CV_32F, 1.0 / FACTOR_16);
        TransmitDisparity(disparityMap.data, timeLeft);
    }
    RETURN_NOERROR;
}

tResult IrToDisparity::OnPinEvent(IPin* pSource, tInt nEventCode,
                                  tInt /*nParam1*/, tInt /*nParam2*/,
                                  IMediaSample* pMediaSample) {
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        if (pSource == &irLeftInputPin || pSource == &irRightInputPin) {
            Process(pMediaSample, pSource);
        }
    } else if (nEventCode == IPinEventSink::PE_MediaTypeChanged) {
        if (pSource == &irLeftInputPin) {
            RETURN_IF_FAILED(updateInputImageFormat(irLeftInputPin.GetFormat(),
                                                    irLeftInputBitmapFormat, irLeftInputImage));
        } else if (pSource == &irRightInputPin) {
            RETURN_IF_FAILED(updateInputImageFormat(irRightInputPin.GetFormat(),
                                                    irRightInputBitmapFormat, irRightInputImage));
        }
    }
    RETURN_NOERROR;
}

tResult IrToDisparity::TransmitDisparity(const void* pData, const tTimeStamp& time) {
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample(&pMediaSample));
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(bitmapFormatDisparity.nSize));
    RETURN_IF_FAILED(pMediaSample->Update(
            time, pData, bitmapFormatDisparity.nSize, IMediaSample::MSF_None));
    RETURN_IF_FAILED(outputPinDisparity.Transmit(pMediaSample));
    RETURN_NOERROR;
}

void IrToDisparity::initSGBM() {
    static const int minDisparity = 0;
    static const int blockSize = 11; //Recommended value: see create doc
    static const int P1 = 8 * blockSize * blockSize; //Recommended value: see create doc
    static const int P2 = 32 * blockSize * blockSize; //Recommended value: see create doc
    static const int disp12MaxDiff = -1; //disable with -1
    static const int preFilterCap = 0;
    static const int uniquenessRatio = 0; // 5-15
    static const int speckleWindowSize = 0;
    static const int speckleRange = 0;
    static const int mode = cv::StereoSGBM::MODE_SGBM_3WAY;

    stereoSGBM = cv::StereoSGBM::create(
            minDisparity, maxDisparity, blockSize, P1, P2,
            disp12MaxDiff, preFilterCap, uniquenessRatio,
            speckleWindowSize, speckleRange, mode);
}
