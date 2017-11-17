#include "VideoPlaybackOpenCV.h"

ADTF_FILTER_PLUGIN(OID_ADTF_FILTER_DEF, OID_ADTF_FILTER_DEF, VideoPlaybackOpenCV)

/**
 * VideoPlayback using OpenCV. If filepath not set, standard camera device is used.
 */
VideoPlaybackOpenCV::VideoPlaybackOpenCV(const tChar *__info) : cFilter(__info) {
    SetPropertyStr("FilePath", " ");
    SetPropertyBool("FilePath" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr ("FilePath" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "Video File (*.*)");
    SetPropertyStr("FilePath" NSSUBPROP_DESCRIPTION, "Path to video");
}

VideoPlaybackOpenCV::~VideoPlaybackOpenCV() {
}

tResult VideoPlaybackOpenCV::Start(__exception) {
    cFilename absoluteFilePath = GetPropertyStr("FilePath");
    ADTF_GET_CONFIG_FILENAME(absoluteFilePath);
    videoCapture.open(string(absoluteFilePath));

    if (!videoCapture.isOpened()) {
        LOG_INFO(adtf_util::cString::Format("File not found, try to use standard camera device"));
        videoCapture.open(0); //Opens standard camera device
    }

    //starting the Thread
    if (m_Thread.GetState() != cKernelThread::TS_Running) {
        m_Thread.Run();
    }
    return cFilter::Start(__exception_ptr);
}

tResult VideoPlaybackOpenCV::Stop(__exception) {
    //suspend the thread
    if (m_Thread.GetState() == cKernelThread::TS_Running) {
        m_Thread.Suspend(tTrue);
    }

    if (videoCapture.isOpened()) {
        videoCapture.release();
    }
    return cFilter::Stop(__exception_ptr);
}

tResult VideoPlaybackOpenCV::Shutdown(tInitStage eStage, ucom::IException **__exception_ptr) {
    if (eStage == StageNormal) {
        m_Thread.Terminate(tTrue);
        m_Thread.Release();

        if (videoCapture.isOpened()) {
            videoCapture.release();
        }
    }
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult VideoPlaybackOpenCV::Init(tInitStage eStage, __exception) {
    //never miss calling the parent implementation first!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid **) &pDescManager, __exception_ptr));

        RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output", IPin::PD_Output, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));

    }
    if (eStage == StageNormal) {
    } else if (eStage == StageGraphReady) {
        //Creating Thread to grab Camera Results in
        m_Thread.Create(cKernelThread::TF_Suspended, static_cast<IKernelThreadFunc *>(this), NULL, 0);
    }

    RETURN_NOERROR;
}

tResult VideoPlaybackOpenCV::ThreadFunc(adtf::cKernelThread *Thread, tVoid *data, tSize size) {

    if (videoCapture.isOpened()) {
        Mat videoFrame;
        videoCapture.read(videoFrame);
        if (!videoFrame.empty()) {
            UpdateOutputImageFormat(videoFrame);
            RETURN_IF_FAILED(transmit(videoFrame));
        }
    }
    //adtf_util::cThread::Sleep(30); Doesnt work...
    std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_FOR_THREAD_SLEEP));
    RETURN_NOERROR;
}

tResult VideoPlaybackOpenCV::transmit(const Mat &outputImage) {
    cImage newImage;
    newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel,
                    m_sOutputFormat.nBytesPerLine, outputImage.data);

    //create the new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));
    //updating media sample
    RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(),
                                          IMediaSample::MSF_None));
    //transmitting
    RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pMediaSample));
    RETURN_NOERROR;
}

void VideoPlaybackOpenCV::UpdateOutputImageFormat(const Mat &outputImage) {
    //check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormat.nSize) {
        Mat2BmpFormat(outputImage, m_sOutputFormat);
        //set output format for output pin
        m_oVideoOutputPin.SetFormat(&m_sOutputFormat, NULL);
    }
}