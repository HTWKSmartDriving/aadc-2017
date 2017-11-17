#ifndef LANE_DETECTION_FILTER_H
#define LANE_DETECTION_FILTER_H

#include "stdafx.h"
#include "../../HTWK_Utils/HTWK_OpenCV_helper.h"
#include "ImageHelper.h"
#include <opencv2/opencv.hpp>
#include "SpatioTemporalLaneDetector.h"
#include "../../HTWK_Types/LaneData.h"
#include "../../HTWK_Types/CameraResolutionData.h"
#include "../../HTWK_Debug/EnableLogs.h"
#define OID_ADTF_FILTER_DEF "htwk.lane_detection"
#define ADTF_FILTER_DESC "HTWK Lane Detection"

#define PROPERTY_START_ROW 380
#define PROPERTY_STOP_ROW 200
#define PROPERTY_STEP_WIDTH 50
#define PROPERTY_FRAME_COUNT 50
#define PROPERTY_MIN 0
#define PROPERTY_MAX_HEIGHT 1080
#define PROPERTY_MAX_FRAMES 500

class LaneDetectionFilter : public adtf::cFilter, IThreadFunc {
ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, adtf::OBJCAT_Auxiliary);

public:
    LaneDetectionFilter(const tChar *__info);

protected:
    cVideoPin videoInputPin;

    cOutputPin outputLanePointsPin;

    cOutputPin outputCameraResolutionDataPin;

    cVideoPin videoOutputPin;
private:
    tBitmapFormat outputBitmapFormat;

    tResult processDebugVideo(Mat &image, const std::vector<tLanePoint> &result);

    void debugLaneDetection(Mat &workingImage, const std::vector<tLanePoint> &result);

    tResult Stop(ucom::IException **__exception_ptr);

    tResult Shutdown(tInitStage eStage, __exception);
    
protected:

    tResult Init(tInitStage eStage, ucom::IException **__exception_ptr);

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

private:
    std::unique_ptr<SpatioTemporalLaneDetector> detector;

    tTimeStamp time = 0;

    tResult ThreadFunc(cThread *Thread, tVoid *data, tSize size);

    tBitmapFormat inputBitmapFormat;

    Mat inputImage;

    tResult transmitCameraHeightAndWidth(const IMediaSample *pMediaSample);

    tResult transmitResult(std::vector<tLanePoint>&);

    void initLaneDetection();

    cThread thread;
};

#endif //LANE_DETECTION_FILTER_H
