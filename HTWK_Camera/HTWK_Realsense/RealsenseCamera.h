#ifndef _HTWK_REALSENSE_FILTER_H_
#define _HTWK_REALSENSE_FILTER_H_

#define OID_ADTF_FILTER_DEF "htwk.camera_realsense"
#define ADTF_FILTER_DESC "HTWK Camera Realsense"

#include "stdafx.h"
#include "../../HTWK_Types/CameraProperties.h"
#include "../../HTWK_Debug/EnableLogs.h"

#define MAXIMUM_DISPARITY_DEFAULT 64
#define PROPERTY_MAX_DISP "Maximum Disparity"
#define FACTOR_16 16
#define PROPERTY_SLEEP_RATE "Sleep time of thread"
#define DEFAULT_SLEEP_RATE 150
#define PROPERTY_EMITTER_ENABLED "Emitter Enabled"
#define PROPERTY_LR_GAIN "LR Gain"
#define PROPERTY_LR_EXPOSURE "LR Exposure"
#define PROPERTY_USE_AUTO_EXPOSURE "Use Auto Exposure"
#define DEFAULT_GAIN 2
#define DEFAULT_EXPOSURE 110
#define FPS 30
#define WIDTH 640
#define HEIGHT 480

class RealsenseCamera : public adtf::cFilter, adtf::IKernelThreadFunc {
ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, OBJCAT_CameraDevice);

public:

    RealsenseCamera(const tChar *__info);

    ~RealsenseCamera();

private:

    tResult Init(tInitStage eStage, ucom::IException **__exception_ptr);

    tResult Shutdown(tInitStage eStage, ucom::IException **__exception_ptr = NULL);

    tResult Start(ucom::IException **__exception_ptr = NULL);

    tResult Stop(ucom::IException **__exception_ptr = NULL);

    tResult ThreadFunc(adtf::cKernelThread *Thread, tVoid *data, tSize size);

    inline tResult TransmitRGB(const void *pData, const tTimeStamp &time);

    inline tResult TransmitDisparity(const void *pData, const tTimeStamp &time);

    inline tResult TransmitInfrared(const void *pData, cVideoPin *pin, const tTimeStamp &time);

    inline tResult transmitCameraProperties(const tTimeStamp &time);

    inline void initSGBM();

    Ptr<StereoSGBM> stereoSGBM;

    cVideoPin outputPinRGB;

    cVideoPin outputPinDisparity;

    cVideoPin outputPinInfraredLeft;

    cVideoPin outputPinInfraredRight;

    cOutputPin outputPinCameraProperties;

    cKernelThread m_Thread;

    tBitmapFormat bitmapFormatRGB;

    tBitmapFormat bitmapFormatDisparity;

    tBitmapFormat bitmapFormatInfrared;

    unique_ptr<rs::context> m_ctx;

    rs::device *m_dev = NULL;

    bool cameraFound = false;

    int maxDisparity = MAXIMUM_DISPARITY_DEFAULT;

    int sleepRate = DEFAULT_SLEEP_RATE;

    bool firstCall = true;
};

#endif // _HTWK_REALSENSE_FILTER_H_
