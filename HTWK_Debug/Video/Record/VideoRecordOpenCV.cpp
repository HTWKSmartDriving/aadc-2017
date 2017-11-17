#include "VideoRecordOpenCV.h"

ADTF_FILTER_PLUGIN(OID_ADTF_FILTER_DEF, OID_ADTF_FILTER_DEF, VideoRecordOpenCV)

/**
 * VideoRecord using OpenCV. If Directory not correct set video writing fails.
 */
VideoRecordOpenCV::VideoRecordOpenCV(const tChar *__info) : cFilter(__info) {
    SetPropertyStr("FilePath", "Example: /user/Desktop/Videos"); //Point to an existing directory!
    SetPropertyBool("FilePath" NSSUBPROP_DIRECTORY, tTrue);
    SetPropertyStr("FilePath" NSSUBPROP_DESCRIPTION, "Directory in which the video should be saved.");

    SetPropertyStr("FileName", "Video");
    SetPropertyStr("FileName" NSSUBPROP_DESCRIPTION, "The filename which will be appended");

    SetPropertyInt("Frame Rate", PROPERTY_FRAME_RATE);
    SetPropertyStr("Frame Rate" NSSUBPROP_DESCRIPTION, "Frame rate of saved video");
    SetPropertyInt("Frame Rate" NSSUBPROP_MIN, PROPERTY_MIN);
    SetPropertyInt("Frame Rate" NSSUBPROP_MAX, PROPERTY_MAX);
}

VideoRecordOpenCV::~VideoRecordOpenCV() {
}

tResult VideoRecordOpenCV::Shutdown(tInitStage eStage, ucom::IException **__exception_ptr) {
    if (eStage == StageNormal) {
        if (videoWriter.isOpened()) {
            videoWriter.release();
        }
    }
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult VideoRecordOpenCV::Init(tInitStage eStage, __exception) {
    //never miss calling the parent implementation first!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid **) &pDescManager, __exception_ptr));

        // Video Input
        RETURN_IF_FAILED(videoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&videoInputPin));

    }
    if (eStage == StageNormal) {
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

tResult VideoRecordOpenCV::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
                                      IMediaSample *pMediaSample) {
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        if (pSource == &videoInputPin) {
            if (!videoWriter.isOpened()) {
                if (inputBitmapFormat.nPixelFormat == IImage::PF_UNKNOWN) {
                    RETURN_IF_FAILED(UpdateInputImageFormat(videoInputPin.GetFormat()));
                }
                initVideoWriter();
            }
            RETURN_IF_FAILED(ProcessVideo(pMediaSample));
        }
    } else if (nEventCode == IPinEventSink::PE_MediaTypeChanged) {
        if (pSource == &videoInputPin) {
            RETURN_IF_FAILED(UpdateInputImageFormat(videoInputPin.GetFormat()));
            initVideoWriter();
        }
    }
    RETURN_NOERROR;
}

void VideoRecordOpenCV::initVideoWriter() {
    cFilename absoluteFilePath = GetPropertyStr("FilePath");
    ADTF_GET_CONFIG_FILENAME(absoluteFilePath);
    absoluteFilePath.AppendTrailingSlash();
    cString timeStamp = cDateTime::GetCurrentDateTime().Format("%X");
    std::string rndAppendix = std::to_string(rand() % 2000 + 1); //To avoid overwriting..
    std::string fileName = GetPropertyStr("FileName");

    string path = string(absoluteFilePath) + fileName + "_" + string(timeStamp) + "_" + rndAppendix + ".avi";
    LOG_INFO(adtf_util::cString::Format("VideoWriter tries to use this path: %s", path.c_str()));
    Size size = Size(inputBitmapFormat.nWidth, inputBitmapFormat.nHeight);
    double fps = GetPropertyInt("Frame Rate");
    bool status = videoWriter.open
            (path, CV_FOURCC('M', 'J', 'P', 'G'), fps, size, true);
    if (!status) {
        LOG_WARNING("VideoWriter couldn't be opened, please specifiy only an existing directory!");
        LOG_WARNING("Example: /user/Desktop/Video");
    }
}

tResult VideoRecordOpenCV::ProcessVideo(IMediaSample *pSample) {
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

            if (!m_inputImage.empty()) {
                if(m_inputImage.channels() == IS_GRAY){
                    Mat output;
                    cvtColor(m_inputImage,output,CV_GRAY2BGR);
                    videoWriter.write(output);
                    RETURN_NOERROR;
                }else{
                    videoWriter.write(m_inputImage);
                }
            }
        }
    }
    RETURN_NOERROR;
}

tResult VideoRecordOpenCV::UpdateInputImageFormat(const tBitmapFormat *pFormat) {
    if (pFormat != NULL) {
        inputBitmapFormat = (*pFormat);
        RETURN_IF_FAILED(BmpFormat2Mat(inputBitmapFormat, m_inputImage));
    }
    RETURN_NOERROR;
}
