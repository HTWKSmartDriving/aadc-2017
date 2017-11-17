#include <thread>
#include "RealsenseCamera.h"

ADTF_FILTER_PLUGIN(OID_ADTF_FILTER_DEF, OID_ADTF_FILTER_DEF, RealsenseCamera);

RealsenseCamera::RealsenseCamera(const tChar *__info) : cFilter(__info) {
    SetPropertyInt(PROPERTY_MAX_DISP, MAXIMUM_DISPARITY_DEFAULT);
    SetPropertyStr(PROPERTY_MAX_DISP NSSUBPROP_DESCRIPTION, "Maximum disparity, must be dividable 16");
    SetPropertyInt(PROPERTY_MAX_DISP NSSUBPROP_MIN, 16);
    SetPropertyInt(PROPERTY_MAX_DISP NSSUBPROP_MAX, 4096);

    SetPropertyInt(PROPERTY_SLEEP_RATE, DEFAULT_SLEEP_RATE);
    SetPropertyStr(PROPERTY_SLEEP_RATE NSSUBPROP_DESCRIPTION, "Thread sleep time in ms");
    SetPropertyInt(PROPERTY_SLEEP_RATE NSSUBPROP_MIN, 0);
    SetPropertyInt(PROPERTY_SLEEP_RATE NSSUBPROP_MAX, 1000);

    SetPropertyBool(PROPERTY_EMITTER_ENABLED, tTrue);

    SetPropertyInt(PROPERTY_LR_GAIN, DEFAULT_GAIN);
    SetPropertyStr(PROPERTY_LR_GAIN NSSUBPROP_DESCRIPTION, "Left-Right-Infrared gain");
    SetPropertyInt(PROPERTY_LR_GAIN NSSUBPROP_MIN, 1);
    SetPropertyInt(PROPERTY_LR_GAIN NSSUBPROP_MAX, 4);

    SetPropertyInt(PROPERTY_LR_EXPOSURE, DEFAULT_EXPOSURE);
    SetPropertyStr(PROPERTY_LR_EXPOSURE NSSUBPROP_DESCRIPTION, "Exposure Time in ms");
    SetPropertyInt(PROPERTY_LR_EXPOSURE NSSUBPROP_MIN, 10);
    SetPropertyInt(PROPERTY_LR_EXPOSURE NSSUBPROP_MAX, 330);

    SetPropertyBool(PROPERTY_USE_AUTO_EXPOSURE, tTrue);
}

RealsenseCamera::~RealsenseCamera() {
}

tResult RealsenseCamera::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst) {
        cObjectPtr<IMediaType> pOutputType;
        RETURN_IF_FAILED(AllocMediaType(&pOutputType, MEDIA_TYPE_CAMERA_PROPERTY, MEDIA_SUBTYPE_CAMERA_PROPERTY,
                                        __exception_ptr));

        RETURN_IF_FAILED(outputPinCameraProperties.Create("Camera_Property", pOutputType, this));
        RETURN_IF_FAILED(RegisterPin(&outputPinCameraProperties));
        pOutputType = NULL;

        outputPinDisparity.Create("Disparity", IPin::PD_Output, static_cast<IPinEventSink *>(this));
        RegisterPin(&outputPinDisparity);

        outputPinInfraredLeft.Create("IR_Left", IPin::PD_Output, static_cast<IPinEventSink *>(this));
        RegisterPin(&outputPinInfraredLeft);

        outputPinInfraredRight.Create("IR_Right", IPin::PD_Output, static_cast<IPinEventSink *>(this));
        RegisterPin(&outputPinInfraredRight);

        outputPinRGB.Create("RGB", IPin::PD_Output, static_cast<IPinEventSink *>(this));
        RegisterPin(&outputPinRGB);

    } else if (eStage == StageNormal) {
        bitmapFormatRGB.nWidth = WIDTH;
        bitmapFormatRGB.nHeight = HEIGHT;
        bitmapFormatRGB.nBitsPerPixel = 24;
        bitmapFormatRGB.nPixelFormat = IImage::PF_RGB_888;
        bitmapFormatRGB.nBytesPerLine = bitmapFormatRGB.nWidth * bitmapFormatRGB.nBitsPerPixel / 8;
        bitmapFormatRGB.nSize = bitmapFormatRGB.nBytesPerLine * bitmapFormatRGB.nHeight;
        bitmapFormatRGB.nPaletteSize = 0;

        bitmapFormatDisparity.nWidth = WIDTH;
        bitmapFormatDisparity.nHeight = HEIGHT;
        bitmapFormatDisparity.nBitsPerPixel = 32;
        bitmapFormatDisparity.nPixelFormat = IImage::PF_GREYSCALE_FLOAT32;
        bitmapFormatDisparity.nBytesPerLine =
                bitmapFormatDisparity.nWidth * bitmapFormatDisparity.nBitsPerPixel / 8;
        bitmapFormatDisparity.nSize = bitmapFormatDisparity.nBytesPerLine * bitmapFormatDisparity.nHeight;
        bitmapFormatDisparity.nPaletteSize = 0;

        bitmapFormatInfrared.nWidth = WIDTH;
        bitmapFormatInfrared.nHeight = HEIGHT;
        bitmapFormatInfrared.nBitsPerPixel = 8;
        bitmapFormatInfrared.nPixelFormat = IImage::PF_GREYSCALE_8;
        bitmapFormatInfrared.nBytesPerLine =
                bitmapFormatInfrared.nWidth * bitmapFormatInfrared.nBitsPerPixel / 8;
        bitmapFormatInfrared.nSize =
                bitmapFormatInfrared.nBytesPerLine * bitmapFormatInfrared.nHeight;
        bitmapFormatInfrared.nPaletteSize = 0;

        outputPinRGB.SetFormat(&bitmapFormatRGB, NULL);
        outputPinDisparity.SetFormat(&bitmapFormatDisparity, NULL);
        outputPinInfraredLeft.SetFormat(&bitmapFormatInfrared, NULL);
        outputPinInfraredRight.SetFormat(&bitmapFormatInfrared, NULL);

    } else if (eStage == StageGraphReady) {
        m_Thread.Create(cKernelThread::TF_Suspended, static_cast<IKernelThreadFunc *>(this), NULL, 0);
    }
    RETURN_NOERROR;
}

tResult RealsenseCamera::Start(__exception) {
    try {
        m_ctx = unique_ptr<rs::context>(new rs::context());
    } catch (const rs::error &e) {
        string error(e.get_failed_function() + "(" + e.get_failed_args() + "): " + e.what());
        LOG_ERROR(error.c_str());
    }
    try {
        if (m_ctx->get_device_count() == 0) {
            LOG_ERROR("Probably no Realsense Camera connected. Connect camera and restart ADTF");
        } else {
            m_dev = m_ctx->get_device(0);
            cameraFound = true;
        }
    } catch (const rs::error &e) { ;
        string error(e.get_failed_function() + "(" + e.get_failed_args() + "): " + e.what());
        LOG_ERROR(error.c_str());
    } catch (const std::exception &e) {
        LOG_ERROR(e.what());
    }
    if (cameraFound) {
        maxDisparity = GetPropertyInt(PROPERTY_MAX_DISP);
        if (maxDisparity % FACTOR_16 != 0) { //SGBM needs maxDisparity dividable 16
            LOG_WARNING("Maximum Disparity must be dividable 16! Standard value is now used");
            maxDisparity = MAXIMUM_DISPARITY_DEFAULT;
        }
        initSGBM();

        sleepRate = GetPropertyInt(PROPERTY_SLEEP_RATE);

        // Setting Emitter Enabled for better Depth Stream in near Range
        m_dev->set_option(rs::option::r200_emitter_enabled, GetPropertyBool(PROPERTY_EMITTER_ENABLED));
        m_dev->set_option(rs::option::r200_lr_gain, GetPropertyInt(PROPERTY_LR_GAIN)); // 1-4, via testing
        if(GetPropertyBool(PROPERTY_USE_AUTO_EXPOSURE)){
            m_dev->set_option(rs::option::r200_lr_auto_exposure_enabled, true);
        }else{
            m_dev->set_option(rs::option::r200_lr_exposure, GetPropertyInt(PROPERTY_LR_EXPOSURE));
        }
        m_dev->enable_stream(rs::stream::color, WIDTH, HEIGHT, rs::format::bgr8, FPS);
        m_dev->enable_stream(rs::stream::infrared, WIDTH, HEIGHT, rs::format::y8, FPS);
        m_dev->enable_stream(rs::stream::infrared2, WIDTH, HEIGHT, rs::format::y8, FPS);

        m_dev->start();
        // Camera warmup - Dropped several first frames to let auto-exposure stabilize
        for (int i = 0; i < 40; i++) {
            m_dev->wait_for_frames();
        }
    }
    //starting the Thread
    if (m_Thread.GetState() != cKernelThread::TS_Running) {
        m_Thread.Run();
    }
    return cFilter::Start(__exception_ptr);
}

tResult RealsenseCamera::Stop(__exception) {
    if (m_Thread.GetState() == cKernelThread::TS_Running) {
        m_Thread.Suspend(tTrue);
    }
    if (cameraFound) {
        m_dev->stop();
        cameraFound = false;
    }
    return cFilter::Stop(__exception_ptr);
}

tResult RealsenseCamera::Shutdown(tInitStage eStage, __exception) {
    if (eStage == StageNormal) {
        m_Thread.Terminate(tTrue);
        m_Thread.Release();
    }
    return cFilter::Shutdown(eStage, __exception_ptr);
}

// Waits for the Streams to send a Frame
tResult RealsenseCamera::ThreadFunc(adtf::cKernelThread *Thread, tVoid *data, tSize size) {
    if (cameraFound) {
        // Waiting for the Frame
        m_dev->wait_for_frames();
        const tTimeStamp time = _clock->GetTime();
        if (firstCall) {
            transmitCameraProperties(time);
            firstCall = false;
        }
        Mat leftIR(Size(WIDTH, HEIGHT), CV_8U, (void *) m_dev->get_frame_data(rs::stream::infrared));
        Mat rightIR(Size(WIDTH, HEIGHT), CV_8U, (void *) m_dev->get_frame_data(rs::stream::infrared2));

        if (outputPinDisparity.IsConnected()) {
            Mat disparityMap;
            stereoSGBM->compute(leftIR, rightIR, disparityMap); //CV_16S
            disparityMap.convertTo(disparityMap, CV_32F, 1.0 / FACTOR_16);
            TransmitDisparity(disparityMap.data, time);
        }
        if (outputPinInfraredLeft.IsConnected()) {
            TransmitInfrared((void *) leftIR.data, &outputPinInfraredLeft, time);
        }
        if (outputPinInfraredRight.IsConnected()) {
            TransmitInfrared((void *) rightIR.data, &outputPinInfraredRight, time);
        }
        if (outputPinRGB.IsConnected()) {
            TransmitRGB(m_dev->get_frame_data(rs::stream::rectified_color), time);
        }
    }
    this_thread::sleep_for(chrono::milliseconds(sleepRate));
    RETURN_NOERROR;
}

tResult RealsenseCamera::TransmitRGB(const void *pData, const tTimeStamp &time) {
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample(&pMediaSample));
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(bitmapFormatRGB.nSize));
    RETURN_IF_FAILED(pMediaSample->Update(time, pData, bitmapFormatRGB.nSize, IMediaSample::MSF_None));
    RETURN_IF_FAILED(outputPinRGB.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult RealsenseCamera::TransmitDisparity(const void *pData, const tTimeStamp &time) {
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample(&pMediaSample));
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(bitmapFormatDisparity.nSize));
    RETURN_IF_FAILED(pMediaSample->Update(time, pData, bitmapFormatDisparity.nSize, IMediaSample::MSF_None));
    RETURN_IF_FAILED(outputPinDisparity.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult RealsenseCamera::TransmitInfrared(const void *pData, cVideoPin *pin, const tTimeStamp &time) {
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample(&pMediaSample));
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(bitmapFormatInfrared.nSize));
    RETURN_IF_FAILED(pMediaSample->Update(time, pData, bitmapFormatInfrared.nSize, IMediaSample::MSF_None));
    RETURN_IF_FAILED(pin->Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult RealsenseCamera::transmitCameraProperties(const tTimeStamp &time) {
    cObjectPtr<IMediaSample> pOutputData;
    if (IS_OK(AllocMediaSample(&pOutputData))) {
        tCameraProperties cProp;
        const rs::intrinsics intrinsicsIR = m_dev->get_stream_intrinsics(rs::stream::infrared);
        cProp.inInfraredLeft.focal = Point2f(intrinsicsIR.fx, intrinsicsIR.fy);
        cProp.inInfraredLeft.principal = Point2f(intrinsicsIR.ppx, intrinsicsIR.ppy);

        const rs::intrinsics intrinsicsRGB = m_dev->get_stream_intrinsics(rs::stream::rectified_color);
        cProp.inRGB.focal = Point2f(intrinsicsRGB.fx, intrinsicsRGB.fy);
        cProp.inRGB.principal = Point2f(intrinsicsRGB.ppx, intrinsicsRGB.ppy);

        cProp.disparity = maxDisparity;

        const rs::extrinsics extrinsics = m_dev->get_extrinsics(rs::stream::infrared, rs::stream::rectified_color);
        memcpy(cProp.exIRtoRGB.translation, extrinsics.translation, sizeof(extrinsics.translation));
        memcpy(cProp.exIRtoRGB.rotation, extrinsics.rotation, sizeof(extrinsics.rotation));
        pOutputData->Update(time, &cProp, sizeof(cProp), 0);
        RETURN_IF_FAILED(outputPinCameraProperties.Transmit(pOutputData));
    }
    RETURN_NOERROR;
}

/**
 * Initalizes the stereo matching class which is used for matching left and right image to an disparity image
 */
void RealsenseCamera::initSGBM() {
    const int minDisparity = 0;
    const int numDisparities = maxDisparity; //Dividable 16
    const int blockSize = 11; //Recommended value: see create doc
    const int P1 = 8 * blockSize * blockSize; //Recommended value: see create doc
    const int P2 = 32 * blockSize * blockSize; //Recommended value: see create doc
    int disp12MaxDiff = -1; //disable with -1
    int preFilterCap = 0;
    int uniquenessRatio = 0; // 5-15
    int speckleWindowSize = 0;
    int speckleRange = 0;
    int mode = StereoSGBM::MODE_SGBM_3WAY;

    stereoSGBM = StereoSGBM::create(minDisparity, numDisparities, blockSize, P1, P2,
                                    disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize,
                                    speckleRange, mode);
}
