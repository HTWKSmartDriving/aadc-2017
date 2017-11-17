#ifndef AADC_VideoPlaybackOpenCV_CLASS_H
#define AADC_VideoPlaybackOpenCV_CLASS_H

#include "stdafx.h"
#include "../../../HTWK_Utils/HTWK_OpenCV_helper.h"
#include <thread>
#include "../../../HTWK_Debug/EnableLogs.h"
#define OID_ADTF_FILTER_DEF "htwk.debug.videoPlaybackOpenCV"
#define ADTF_FILTER_DESC "HTWK DEBUG Video Playback OpenCV"
#define DELAY_FOR_THREAD_SLEEP 30

class VideoPlaybackOpenCV : public adtf::cFilter, adtf::IKernelThreadFunc {
public:
ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, adtf::OBJCAT_CameraDevice);

public:

    VideoPlaybackOpenCV(const tChar *__info);

    ~VideoPlaybackOpenCV();

    tResult Init(tInitStage eStage, ucom::IException **__exception_ptr);

    tResult Start(ucom::IException **__exception_ptr = NULL);

    tResult Stop(ucom::IException **__exception_ptr = NULL);

    tResult Shutdown(tInitStage eStage, ucom::IException **__exception_ptr = NULL);

private:
    void UpdateOutputImageFormat(const cv::Mat &outputImage);

    tResult transmit(const Mat &outputImage);

    cVideoPin m_oVideoOutputPin;

    tBitmapFormat m_sOutputFormat;

    tResult ThreadFunc(adtf::cKernelThread *Thread, tVoid *data, tSize size);

    cKernelThread m_Thread;

    cv::VideoCapture videoCapture;
};

#endif