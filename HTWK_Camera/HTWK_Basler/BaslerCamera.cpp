#include "BaslerCamera.h"

ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, BaslerCamera)

BaslerCamera::BaslerCamera(const tChar *__info) : cFilter(__info) {
    SetPropertyInt("Stream Width", WIDTH);
    SetPropertyStr("Stream Width" NSSUBPROP_DESCRIPTION, "Width of the Stream");
    SetPropertyBool("Stream Width" NSSUBPROP_READONLY, tTrue);

    SetPropertyInt("Stream Height", HEIGHT);
    SetPropertyStr("Stream Height" NSSUBPROP_DESCRIPTION, "Height of the Stream");
    SetPropertyBool("Stream Height" NSSUBPROP_READONLY, tTrue);

    SetPropertyFloat("Brightness", 0.3);
    SetPropertyStr("Brightness" NSSUBPROP_DESCRIPTION, "Target Brightness for Auto Brightness of Camera");

    SetPropertyInt("Frame Rate", 30);
    SetPropertyStr("Frame Rate" NSSUBPROP_DESCRIPTION, "Frames per Second");
    SetPropertyInt("Frame Rate" NSSUBPROP_MIN, 1);
    SetPropertyInt("Frame Rate" NSSUBPROP_MAX, 50);
}

BaslerCamera::~BaslerCamera() {
}

tResult BaslerCamera::Start(__exception) {
    if (!pylonInitializeCalled) {
        //Initializing Pylon5 only once
        try {
            Pylon::PylonInitialize();
        }
        catch (GenICam::GenericException &e) {
            THROW_ERROR_DESC(ERR_NOT_CONNECTED, cString::Format("Pylon not Initialized: %s", e.GetDescription()));
        }
        try {
            // Only look for cameras supported by Camera_t.
            Pylon::CDeviceInfo info;
            info.SetDeviceClass(Pylon::CBaslerUsbInstantCamera::DeviceClass());
            // Create an instant camera object with the first found camera device that matches the specified device class.
            //Attaching camera to camera instance member
            m_camera.Attach(Pylon::CTlFactory::GetInstance().CreateFirstDevice(info));
        }
        catch (GenICam::GenericException &e) {
            THROW_ERROR_DESC(ERR_NOT_CONNECTED,
                             cString::Format("Camera could not be initialized: %s", e.GetDescription()));
        }
        pylonInitializeCalled = true;
    }

    try {
        m_camera.Open();
        //Pixel Format for camera Output
        m_camera.PixelFormat.SetValue(Basler_UsbCameraParams::PixelFormat_RGB8);
        m_camera.Width.SetValue(WIDTH);
        m_camera.Height.SetValue(HEIGHT);
        //Setting Camera options from Properties
        m_camera.AutoTargetBrightness.SetValue(GetPropertyFloat("Brightness"));
        m_camera.AcquisitionFrameRate.SetValue(GetPropertyInt("Frame Rate"));
    }
    catch (GenICam::GenericException &e) {
        THROW_ERROR_DESC(ERR_NOT_CONNECTED,
                         cString::Format("Camera could not be initialized: %s", e.GetDescription()));
    }
    try {
        m_camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    }
    catch (GenICam::GenericException &e) {
        THROW_ERROR_DESC(ERR_FAILED, cString::Format("Camera cannot start grabbing: %s", e.GetDescription()));
    }
    //starting the Thread
    if (m_Thread.GetState() != cKernelThread::TS_Running) {
        m_Thread.Run();
    }
    return cFilter::Start(__exception_ptr);
}

tResult BaslerCamera::Stop(__exception) {
    //suspend the thread
    if (m_Thread.GetState() == cKernelThread::TS_Running) {
        m_Thread.Suspend(tTrue);
    }
    // stops grabbing
    m_camera.StopGrabbing();
    return cFilter::Stop(__exception_ptr);
}

tResult BaslerCamera::Shutdown(tInitStage eStage, ucom::IException **__exception_ptr) {
    if (eStage == StageNormal) {
        m_Thread.Terminate(tTrue);
        m_Thread.Release();
        if (m_camera.IsOpen()) {
            m_camera.Close();
            m_camera.DetachDevice();
            m_camera.DestroyDevice();
        }
        try {
            pylonInitializeCalled = false;
            Pylon::PylonTerminate();
        }
        catch (GenICam::GenericException &e) {
            THROW_ERROR_DESC(ERR_FAILED, cString::Format("Pylon not deinitialized: %s", e.GetDescription()));
        }
    }
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult BaslerCamera::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
    if (eStage == StageFirst) {
        m_videooutputRGB.Create("RGB", IPin::PD_Output, static_cast<IPinEventSink *>(this));
        RegisterPin(&m_videooutputRGB);
    }
    if (eStage == StageNormal) {
        //Setting Bitmapformat for Output
        m_BitmapFormatRGBOut.nPixelFormat = adtf_util::IImage::PF_BGR_888;
        m_BitmapFormatRGBOut.nBitsPerPixel = 24;
        m_BitmapFormatRGBOut.nWidth = WIDTH;
        m_BitmapFormatRGBOut.nHeight = HEIGHT;
        m_BitmapFormatRGBOut.nBytesPerLine = m_BitmapFormatRGBOut.nWidth * m_BitmapFormatRGBOut.nBitsPerPixel / 8;
        m_BitmapFormatRGBOut.nSize = m_BitmapFormatRGBOut.nBytesPerLine * m_BitmapFormatRGBOut.nHeight;
        m_BitmapFormatRGBOut.nPaletteSize = 0;
        m_videooutputRGB.SetFormat(&m_BitmapFormatRGBOut, NULL);
    } else if (eStage == StageGraphReady) {
        m_Thread.Create(cKernelThread::TF_Suspended, static_cast<IKernelThreadFunc *>(this), NULL, 0);
    }
    RETURN_NOERROR;
}

tResult BaslerCamera::ThreadFunc(adtf::cKernelThread *Thread, tVoid *data, tSize size) {
    Pylon::CGrabResultPtr ptrGrabResult;
    if (m_camera.IsOpen() && m_camera.IsGrabbing()) {
        try {
            m_camera.RetrieveResult(1000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        }
        catch (GenICam::GenericException &e) {
            LOG_ERROR(cString::Format("An exception occurred.  %s", e.GetDescription()));
            RETURN_NOERROR;
        }
        if (ptrGrabResult->GrabSucceeded() && m_BitmapFormatRGBOut.nSize == tInt32(ptrGrabResult->GetImageSize())) {
            transmit(ptrGrabResult->GetBuffer(), ptrGrabResult->GetImageSize());
        }
    }
    ptrGrabResult.Release();
    RETURN_NOERROR;
}

tResult BaslerCamera::transmit(const void *pData, const size_t size) {
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));
    RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), pData, tInt(size), IMediaSample::MSF_None));
    RETURN_IF_FAILED(m_videooutputRGB.Transmit(pMediaSample));
    RETURN_NOERROR;
}
