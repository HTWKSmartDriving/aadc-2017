#ifndef AADC_VideoRecordOpenCV_CLASS_H
#define AADC_VideoRecordOpenCV_CLASS_H

#include "stdafx.h"
#include "../../../HTWK_Utils/HTWK_OpenCV_helper.h"
#include "../../../HTWK_Debug/EnableLogs.h"
#define OID_ADTF_FILTER_DEF "htwk.debug.videoRecordOpenCV"
#define ADTF_FILTER_DESC "HTWK DEBUG Video Record OpenCV"

#define PROPERTY_FRAME_RATE 30
#define PROPERTY_MIN 20
#define PROPERTY_MAX 60

#define IS_GRAY 1

class VideoRecordOpenCV : public adtf::cFilter{
public:
ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, adtf::OBJCAT_CameraDevice);

public:

    VideoRecordOpenCV(const tChar *__info);

    ~VideoRecordOpenCV();

    tResult Init(tInitStage eStage, ucom::IException **__exception_ptr);

    tResult Shutdown(tInitStage eStage, ucom::IException **__exception_ptr = NULL);

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

private:

    cVideoPin videoInputPin;

    tBitmapFormat inputBitmapFormat;

    cv::VideoWriter videoWriter;

    Mat m_inputImage;

    tResult UpdateInputImageFormat(const tBitmapFormat *pFormat);

    tResult ProcessVideo(IMediaSample *pSample);

    void initVideoWriter();
};

#endif