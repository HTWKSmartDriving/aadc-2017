#ifndef HTWK_BASLER_CAMERA_H
#define HTWK_BASLER_CAMERA_H

#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <adtf_graphics.h>
#include "../../HTWK_Debug/EnableLogs.h"
#define OID_ADTF_FILTER_DEF "htwk.camera_basler"
#define ADTF_FILTER_DESC "HTWK Camera Basler"

class BaslerCamera : public adtf::cFilter, adtf::IKernelThreadFunc {
ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, OBJCAT_CameraDevice);

public:

    BaslerCamera(const tChar *__info);

    ~BaslerCamera();

    tResult Init(tInitStage eStage, ucom::IException **__exception_ptr);

    tResult Start(ucom::IException **__exception_ptr = NULL);

    tResult Stop(ucom::IException **__exception_ptr = NULL);

    tResult Shutdown(tInitStage eStage, ucom::IException **__exception_ptr = NULL);

private:

    tResult transmit(const void *pData, const size_t size);

    cVideoPin m_videooutputRGB;

    tBitmapFormat m_BitmapFormatRGBOut;

    tResult ThreadFunc(adtf::cKernelThread *Thread, tVoid *data, tSize size);

    Pylon::CBaslerUsbInstantCamera m_camera;

    cKernelThread m_Thread;

    bool pylonInitializeCalled = false;

    constexpr static int WIDTH = 1280;
    constexpr static int HEIGHT = 960;
};

#endif
