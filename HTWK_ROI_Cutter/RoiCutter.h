#ifndef ROI_CUTTER_FILTER_H
#define ROI_CUTTER_FILTER_H

#include "stdafx.h"
#include <ADTF_OpenCV_helper.h>
#include "../HTWK_Debug/EnableLogs.h"
#define OID_ADTF_FILTER_DEF "htwk.roi_cutter"
#define ADTF_FILTER_DESC "HTWK ROI Cutter"

#define PROPERTY_CUT_OFF_LINE "Cut-Off line"

class ROICutter : public adtf::cFilter {
ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, adtf::OBJCAT_Auxiliary);

public:
    ROICutter(const tChar *__info);

protected:
    cVideoPin videoInputPin;

    cVideoPin videoOutputPin;
private:
    tBitmapFormat outputBitmapFormat;

    void UpdateOutputImageFormat(const Mat &outputImage);

protected:

    tResult Init(tInitStage eStage, ucom::IException **__exception_ptr);

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

private:
    tResult UpdateInputImageFormat(const tBitmapFormat *pFormat);

    tResult ProcessVideo(IMediaSample *pSample);

    tResult processOutputVideo(Mat &image);

    tBitmapFormat inputBitmapFormat;

    Mat m_inputImage;
};

#endif //ROI_CUTTER_FILTER_H
