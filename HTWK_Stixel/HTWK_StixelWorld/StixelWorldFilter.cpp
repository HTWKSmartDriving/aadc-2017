#include "StixelWorldFilter.h"

ADTF_FILTER_PLUGIN(OID_ADTF_FILTER_DEF, OID_ADTF_FILTER_DEF, StixelWorldFilter)

StixelWorldFilter::StixelWorldFilter(const tChar *__info) : cFilter(__info) {
    SetPropertyInt(PROPERTY_STIXEL_WIDTH, STIXEL_WIDTH_DEFAULT);
    SetPropertyStr(PROPERTY_STIXEL_WIDTH NSSUBPROP_DESCRIPTION, "Width of Stixel");
    SetPropertyInt(PROPERTY_STIXEL_WIDTH NSSUBPROP_MIN, 1);
    SetPropertyInt(PROPERTY_STIXEL_WIDTH NSSUBPROP_MAX, 100);

    SetPropertyInt(PROPERTY_FREE_P1, FREE_SPACE_P1);
    SetPropertyStr(PROPERTY_FREE_P1 NSSUBPROP_DESCRIPTION, "Parameter one for calculation");
    SetPropertyInt(PROPERTY_FREE_P1 NSSUBPROP_MIN, 1);
    SetPropertyInt(PROPERTY_FREE_P1 NSSUBPROP_MAX, 500);
    SetPropertyBool(PROPERTY_FREE_P1 NSSUBPROP_HIDDEN, tTrue);

    SetPropertyInt(PROPERTY_FREE_P2, FREE_SPACE_P2);
    SetPropertyStr(PROPERTY_FREE_P2 NSSUBPROP_DESCRIPTION, "Parameter two for calculation");
    SetPropertyInt(PROPERTY_FREE_P2 NSSUBPROP_MIN, 1);
    SetPropertyInt(PROPERTY_FREE_P2 NSSUBPROP_MAX, 500);
    SetPropertyBool(PROPERTY_FREE_P2 NSSUBPROP_HIDDEN, tTrue);

    SetPropertyInt(PROPERTY_FREE_JUMP, FREE_SPACE_MAX_PIXEL_JUMP);
    SetPropertyStr(PROPERTY_FREE_JUMP NSSUBPROP_DESCRIPTION, "Maximum pixel jump for lower path");
    SetPropertyInt(PROPERTY_FREE_JUMP NSSUBPROP_MIN, 0);
    SetPropertyInt(PROPERTY_FREE_JUMP NSSUBPROP_MAX, 1000);

    SetPropertyFloat(PROPERTY_OBJECT_HEIGHT, FREE_SPACE_OBJECT_HEIGHT);
    SetPropertyStr(PROPERTY_OBJECT_HEIGHT NSSUBPROP_DESCRIPTION, "Size of object in meter");
    SetPropertyFloat(PROPERTY_OBJECT_HEIGHT NSSUBPROP_MIN, 0);
    SetPropertyFloat(PROPERTY_OBJECT_HEIGHT NSSUBPROP_MAX, 100);

    SetPropertyInt(PROPERTY_HEIGHT_SEG_CS, HEIGHT_SEG_C_S);
    SetPropertyStr(PROPERTY_HEIGHT_SEG_CS NSSUBPROP_DESCRIPTION, "Cost smoothnes for Height Semgentation");
    SetPropertyInt(PROPERTY_HEIGHT_SEG_CS NSSUBPROP_MIN, 1);
    SetPropertyInt(PROPERTY_HEIGHT_SEG_CS NSSUBPROP_MAX, 10);
    SetPropertyBool(PROPERTY_HEIGHT_SEG_CS NSSUBPROP_HIDDEN, tTrue);

    SetPropertyFloat(PROPERTY_HEIGHT_SEG_NZ, HEIGHT_SEG_N_Z);
    SetPropertyStr(PROPERTY_HEIGHT_SEG_NZ NSSUBPROP_DESCRIPTION, "Neightbours in Z-Direction for Height Segmentation");
    SetPropertyFloat(PROPERTY_HEIGHT_SEG_NZ NSSUBPROP_MIN, 0.1);
    SetPropertyFloat(PROPERTY_HEIGHT_SEG_NZ NSSUBPROP_MAX, 10);

    SetPropertyFloat(PROPERTY_HEIGHT_SEG_DELTAZ, HEIGHT_SEG_DELTA_Z);
    SetPropertyStr(PROPERTY_HEIGHT_SEG_DELTAZ NSSUBPROP_DESCRIPTION, "deltaZ for calculation");
    SetPropertyFloat(PROPERTY_HEIGHT_SEG_DELTAZ NSSUBPROP_MIN, 0);
    SetPropertyFloat(PROPERTY_HEIGHT_SEG_DELTAZ NSSUBPROP_MAX, 100);

    SetPropertyInt(PROPERTY_HEIGHT_SEG_MAX_JUMP, HEIGHT_SEG_MAX_PIXEL_JUMP);
    SetPropertyStr(PROPERTY_HEIGHT_SEG_MAX_JUMP NSSUBPROP_DESCRIPTION, "Maximum pixel jump for upper path");
    SetPropertyInt(PROPERTY_HEIGHT_SEG_MAX_JUMP NSSUBPROP_MIN, 0);
    SetPropertyInt(PROPERTY_HEIGHT_SEG_MAX_JUMP NSSUBPROP_MAX, 1000);
}

StixelWorldFilter::~StixelWorldFilter() {}

tResult StixelWorldFilter::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {

        cObjectPtr<IMediaType> pInputType;
        RETURN_IF_FAILED(AllocMediaType(&pInputType, MEDIA_TYPE_CAMERA_PROPERTY, MEDIA_SUBTYPE_CAMERA_PROPERTY,
                                        __exception_ptr));
        // create and register the input pin
        RETURN_IF_FAILED(inputPinCameraProperty.Create("Camera_Property", pInputType, this));
        RETURN_IF_FAILED(RegisterPin(&inputPinCameraProperty));
        pInputType = NULL;

        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid **) &pDescManager, __exception_ptr));
        RETURN_IF_FAILED(rgbInputPin.Create("RGB", IPin::PD_Input, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&rgbInputPin));
        // Video Input
        RETURN_IF_FAILED(
                disparityInputPin.Create("Disparity", IPin::PD_Input, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&disparityInputPin));

        RETURN_IF_FAILED(leftInputPin.Create("Debug_IR_Left", IPin::PD_Input, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&leftInputPin));

        // create and register output pins
        cObjectPtr<IMediaType> pOutputType;
        RETURN_IF_FAILED(
                AllocMediaType(&pOutputType, MEDIA_TYPE_STIXEL_DATA, MEDIA_SUBTYPE_STIXEL_DATA, __exception_ptr));

        // create and register the output pin
        RETURN_IF_FAILED(stixelOutputPin.Create("Stixels", pOutputType, this));
        RETURN_IF_FAILED(RegisterPin(&stixelOutputPin));

        RETURN_IF_FAILED(
                rgbOutputPin.Create("RGB_Output", IPin::PD_Output, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&rgbOutputPin));

        RETURN_IF_FAILED(
                debugStixelOutputPin.Create("Debug_Stixel_Video", IPin::PD_Output, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&debugStixelOutputPin));

        RETURN_IF_FAILED(debugDisparityOutputPin.Create("Debug_Disparity_Map", IPin::PD_Output,
                                                        static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&debugDisparityOutputPin));

    } else if (eStage == StageGraphReady) {
        cObjectPtr<IMediaType> pType;
        cObjectPtr<IMediaTypeVideo> pTypeVideo;

        RETURN_IF_FAILED(rgbInputPin.GetMediaType(&pType));
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid **) &pTypeVideo));
        updateInputImageFormat(pTypeVideo->GetFormat(), rgbInputBitmapFormat, rgbInputImage);

        RETURN_IF_FAILED(leftInputPin.GetMediaType(&pType));
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid **) &pTypeVideo));
        updateInputImageFormat(pTypeVideo->GetFormat(), leftInputBitmapFormat, leftInputImage);

        RETURN_IF_FAILED(disparityInputPin.GetMediaType(&pType));
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid **) &pTypeVideo));
        updateInputImageFormat(pTypeVideo->GetFormat(), disparityInputBitmapFormat, disparityInputImage);

        RETURN_IF_FAILED(thread.Create(NULL, 0, cThread::TS_Suspended, static_cast<IThreadFunc *>(this)));
    }
    RETURN_NOERROR;
}

tResult StixelWorldFilter::Stop(ucom::IException **__exception_ptr) {
    if (thread.GetState() == cThread::TS_Running) {
        thread.Suspend(tTrue);
    }
    return cFilter::Stop(__exception_ptr);
}

tResult StixelWorldFilter::Shutdown(tInitStage eStage, __exception) {
    if (eStage == StageNormal) {
        thread.Terminate(tTrue);
        thread.Release();
    }
    return cFilter::Shutdown(eStage, __exception_ptr);
}

/**
 * Loads all properties and inits stixelWorld class
 */
void StixelWorldFilter::initStixelWorld(const tCameraProperties &cProp) {
    const int stixelWidth = GetPropertyInt(PROPERTY_STIXEL_WIDTH);

    FreeSpaceParameter freeSpaceParameter;
    freeSpaceParameter.parameterOne = GetPropertyInt(PROPERTY_FREE_P1);
    freeSpaceParameter.parameterTwo = GetPropertyInt(PROPERTY_FREE_P2);
    freeSpaceParameter.maxPixelJump = GetPropertyInt(PROPERTY_FREE_JUMP);
    freeSpaceParameter.objectHeight = GetPropertyFloat(PROPERTY_OBJECT_HEIGHT);

    HeightSegmentationParameter heightSegmentationParameter;
    heightSegmentationParameter.C_s = GetPropertyInt(PROPERTY_HEIGHT_SEG_CS);
    heightSegmentationParameter.N_z = GetPropertyFloat(PROPERTY_HEIGHT_SEG_NZ);
    heightSegmentationParameter.deltaZ = GetPropertyFloat(PROPERTY_HEIGHT_SEG_DELTAZ);
    heightSegmentationParameter.maxPixelJump = GetPropertyInt(PROPERTY_HEIGHT_SEG_MAX_JUMP);

    stixelWorld = unique_ptr<WorldOfStixel>(new WorldOfStixel(cProp, stixelWidth, freeSpaceParameter,
                                                              heightSegmentationParameter));
}

tResult StixelWorldFilter::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
                                      IMediaSample *pMediaSample) {
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        if (pSource == &inputPinCameraProperty && !stixelWorld) {
            {
                __sample_read_lock(pMediaSample, tCameraProperties, pData);
                tCameraProperties recievedSample = *pData;
                initStixelWorld(recievedSample);
            }
        } else if (stixelWorld && thread.GetState() == cThread::TS_Suspended) {
            if (pSource == &disparityInputPin && !disparityUpdated) {
                RETURN_IF_FAILED(bufferToMat(disparityInputPin.GetFormat(), pMediaSample, disparityInputImage,
                                             disparityInputBitmapFormat));
                disparityTimeStamp = pMediaSample->GetTime();
                disparityUpdated = true;
            } else if (pSource == &rgbInputPin && !rgbImageUpdated) {
                RETURN_IF_FAILED(
                        bufferToMat(rgbInputPin.GetFormat(), pMediaSample, rgbInputImage, rgbInputBitmapFormat));
                rgbImageUpdated = true;
            } else if (pSource == &leftInputPin && !leftImageUpdated) {
                RETURN_IF_FAILED(
                        bufferToMat(leftInputPin.GetFormat(), pMediaSample, leftInputImage, leftInputBitmapFormat));
                leftImageUpdated = true;
            }
            if ((!leftInputPin.IsConnected() && disparityUpdated) ||
                (leftInputPin.IsConnected() && disparityUpdated && leftImageUpdated)) {
                if (rgbImageUpdated) {
                    //All updated, previous run is finished
                    RETURN_IF_FAILED(thread.Run());
                }
            }
        } else if (!stixelWorld) {
            LOG_ERROR("StixelWorld wasn't initalized");
        }
    } else if (nEventCode == IPinEventSink::PE_MediaTypeChanged) {
        if (pSource == &disparityInputPin) {
            RETURN_IF_FAILED(updateInputImageFormat(disparityInputPin.GetFormat(), disparityInputBitmapFormat,
                                                    disparityInputImage));
        } else if (pSource == &leftInputPin) {
            RETURN_IF_FAILED(updateInputImageFormat(leftInputPin.GetFormat(), leftInputBitmapFormat, leftInputImage));
        } else if (pSource == &rgbInputPin) {
            RETURN_IF_FAILED(updateInputImageFormat(rgbInputPin.GetFormat(), rgbInputBitmapFormat, rgbInputImage));
        }
    }
    RETURN_NOERROR;
}

/**
 * calcs Stixel in an extra thread, also transmits results in this thread
 */
tResult StixelWorldFilter::ThreadFunc(cThread *Thread, tVoid *data, tSize size) {
    stixelWorld->compute(disparityInputImage);
    RETURN_IF_FAILED(transmitVideo(rgbInputImage, rgbOutputPin, rgbOutputBitmapFormat));
    RETURN_IF_FAILED(transmitStixelWorld(stixelWorld->getComputedStixel()));

    if (debugStixelOutputPin.IsConnected() && leftImageUpdated) {
        Mat stixelImage = stixelWorld->drawStixelWorld(leftInputImage);
        RETURN_IF_FAILED(transmitVideo(stixelImage, debugStixelOutputPin, debugStixelOutputBitmapFormat));
    }
    if (debugDisparityOutputPin.IsConnected()) {
        double min, max;
        cv::minMaxLoc(disparityInputImage, &min, &max);
        Mat outputDisparity;
        disparityInputImage.convertTo(outputDisparity, CV_16UC1, UINT16_MAX / (max - min));
        RETURN_IF_FAILED(
                transmitVideo(outputDisparity, debugDisparityOutputPin, debugDisparityOutputBitmapFormat));
    }
    disparityUpdated = false;
    leftImageUpdated = false;
    rgbImageUpdated = false;
    RETURN_IF_FAILED(thread.Suspend()); //Ohne Suspend, wird ThreadFunc nicht erneut ausgefuehrt... ?
    RETURN_NOERROR;
}

tResult StixelWorldFilter::transmitStixelWorld(const vector<tStixel> &stixels) {
    if (!stixelOutputPin.IsConnected()) {
        RETURN_NOERROR;
    }
    cObjectPtr<IMediaSample> pOutputStixel;
    if (IS_OK(AllocMediaSample(&pOutputStixel))) {
        pOutputStixel->Update(disparityTimeStamp, &stixels.front(), sizeof(tStixel) * stixels.size(), 0);
        RETURN_IF_FAILED(stixelOutputPin.Transmit(pOutputStixel));
    }
    RETURN_NOERROR;
}

tResult StixelWorldFilter::transmitVideo(const Mat &image, cVideoPin &pin, tBitmapFormat &bmp) {
    updateOutputImageFormat(image, bmp, pin);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(bmp.nSize));
    RETURN_IF_FAILED(pMediaSample->Update(disparityTimeStamp, image.data, bmp.nSize, IMediaSample::MSF_None));
    RETURN_IF_FAILED(pin.Transmit(pMediaSample));
    RETURN_NOERROR;
}
