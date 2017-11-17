#include "RoiCutter.h"

ADTF_FILTER_PLUGIN(OID_ADTF_FILTER_DEF, OID_ADTF_FILTER_DEF, ROICutter)

ROICutter::ROICutter(const tChar *__info) : cFilter(__info) {
    SetPropertyInt(PROPERTY_CUT_OFF_LINE, 470);
    SetPropertyStr(PROPERTY_CUT_OFF_LINE NSSUBPROP_DESCRIPTION, "Up to which line the image should exist");
    SetPropertyInt(PROPERTY_CUT_OFF_LINE NSSUBPROP_MIN, 0);
    SetPropertyInt(PROPERTY_CUT_OFF_LINE NSSUBPROP_MAX, 960);
}

tResult ROICutter::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid **) &pDescManager, __exception_ptr));

        // Video Input
        RETURN_IF_FAILED(videoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&videoInputPin));

        // create and register debug video output pin
        RETURN_IF_FAILED(
                videoOutputPin.Create("Video_Output", IPin::PD_Output, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&videoOutputPin));

    } else if (eStage == StageGraphReady) {
        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(videoInputPin.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid **) &pTypeVideo));

        // set the image format of the input video pin
        UpdateInputImageFormat(pTypeVideo->GetFormat());
    }
    RETURN_NOERROR;
}

tResult ROICutter::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
                              IMediaSample *pMediaSample) {
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        if (pSource == &videoInputPin) {
            if (inputBitmapFormat.nPixelFormat == IImage::PF_UNKNOWN) {
                RETURN_IF_FAILED(UpdateInputImageFormat(videoInputPin.GetFormat()));
            }

            RETURN_IF_FAILED(ProcessVideo(pMediaSample));
        }
    }
    RETURN_NOERROR;
}

tResult ROICutter::ProcessVideo(IMediaSample *pSample) {
    RETURN_IF_POINTER_NULL(pSample);
    // new image for result
    const tVoid *l_pSrcBuffer;

    //receiving data from input sample, and saving to TheInputImage
    if (IS_OK(pSample->Lock(&l_pSrcBuffer))) {
        //convert to mat, be sure to select the right pixelformat
        if (tInt32(m_inputImage.total() * m_inputImage.elemSize()) == inputBitmapFormat.nSize) {
            //copy the data to matrix
            memcpy(m_inputImage.data, l_pSrcBuffer, size_t(inputBitmapFormat.nSize));
            pSample->Unlock(l_pSrcBuffer);

            if (m_inputImage.empty()) {
                RETURN_NOERROR;
            }

            int cropAtLine = GetPropertyInt(PROPERTY_CUT_OFF_LINE);
            cv::Mat croppedMatRef = m_inputImage(
                    Rect(0, cropAtLine, m_inputImage.cols, m_inputImage.rows - cropAtLine));

            //cv::cvtColor(croppedMatRef, croppedMatRef, COLOR_BGR2RGB);

            RETURN_IF_FAILED(processOutputVideo(croppedMatRef));
        }
    }
    RETURN_NOERROR;
}

tResult ROICutter::UpdateInputImageFormat(const tBitmapFormat *pFormat) {
    if (pFormat != NULL) {
        //update member variable
        inputBitmapFormat = (*pFormat);
        //create the input matrix
        RETURN_IF_FAILED(BmpFormat2Mat(inputBitmapFormat, m_inputImage));
    }
    RETURN_NOERROR;
}

void ROICutter::UpdateOutputImageFormat(const cv::Mat &outputImage) {
    //check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != outputBitmapFormat.nSize) {
        Mat2BmpFormat(outputImage, outputBitmapFormat);
        //set output format for output pin
        videoOutputPin.SetFormat(&outputBitmapFormat, NULL);
    }
}

tResult ROICutter::processOutputVideo(Mat &image) {
    UpdateOutputImageFormat(image);

    cImage newImage;
    newImage.Create(outputBitmapFormat.nWidth, outputBitmapFormat.nHeight, outputBitmapFormat.nBitsPerPixel,
                    outputBitmapFormat.nBytesPerLine, image.data);

    //create the new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));
    //updating media sample
    RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(),
                                          IMediaSample::MSF_None));
    //transmitting
    RETURN_IF_FAILED(videoOutputPin.Transmit(pMediaSample));
    RETURN_NOERROR;
}