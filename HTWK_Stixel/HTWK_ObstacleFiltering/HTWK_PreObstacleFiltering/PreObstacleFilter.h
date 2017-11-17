#ifndef PRE_OBSTACLE_FILTER_H
#define PRE_OBSTACLE_FILTER_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

using namespace adtf;

#include <adtf_graphics.h>

using namespace adtf_graphics;

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

#include "ImageHelper.h"
#include "../../../HTWK_Types/ObstacleData.h"
#include "../../../HTWK_Utils/VectorHelper.h"
#include "../../../HTWK_Types/InitalRoadSign.h"

#define PROPERTY_TIME_OUT "Timeout Inital Sign"
#define PROPERTY_DEFAULT_TIME_OUT 5.0
#define PROPERTY_THRESHOLD "Threshold"
#define PROPERTY_DEFAULT_THRESHOLD 100
#define PROPERTY_VOTE_NOT_ROAD "Percent of needed White Pixels"
#define PROPERTY_DEFAULT_VOTE_NOT_ROAD 0.33
#define PROPERTY_SIGN_RADIUS "Sign Radius"
#define PROPERTY_DEFAULT_SIGN_RADIUS 0.20
#define PROPERTY_FILTER_ROAD "Filter Road"
#define PROPERTY_DEFAULT_FILTER_ROAD tFalse
//#define PROPERTY_COLOR_EQUALIZE "Equalize Color"
//#define PROPERTY_DEFAULT_COLOR_EQUALIZE tFalse
#define OID_ADTF_FILTER_DEF "htwk.preObstacleFilter"
#define ADTF_FILTER_DESC "HTWK Pre Obstacle Filter"

class PreObstacleFilter : public adtf::cFilter {
ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, adtf::OBJCAT_DataFilter);

public:
    PreObstacleFilter(const tChar *__info);

private:
    tResult Start(ucom::IException **__exception_ptr);

    void filterObstacleMap(const vector<tObstacleData> &inputVec, const tTimeStamp &time);

    tResult transmitVideo(const Mat &image, cVideoPin &pin, tBitmapFormat &bmp, const tTimeStamp &time);

    cInputPin obstacleInputPin;
    cOutputPin obstacleOutputPin;
    cInputPin inputInitalRoadSignsPin;
    cVideoPin rgbInputPin;
    cVideoPin rgbOutputPin;

    tBitmapFormat rgbInputBitmapFormat;
    tBitmapFormat rgbOutputBitmapFormat;
    Mat rgbImage;
    bool imageUpdated = false;
    bool obstaclesUpdated = false;
    bool signsUpdated = false;
    tTimeStamp startWaitTimeSign = 0;
    double timeOutWaitSign = PROPERTY_DEFAULT_TIME_OUT;
    vector<tObstacleData> recObstacleMap;

    tResult Init(tInitStage eStage, ucom::IException **__exception_ptr);

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

    int threshold = PROPERTY_DEFAULT_THRESHOLD;
    float voteToNotRoad = PROPERTY_DEFAULT_VOTE_NOT_ROAD;
    float signRadius = PROPERTY_DEFAULT_SIGN_RADIUS;
    bool isRoadFilteringEnabled = PROPERTY_DEFAULT_FILTER_ROAD;
    //bool isColorEqualizationEnabled = PROPERTY_DEFAULT_COLOR_EQUALIZE;
    vector<Point2f> signMap;

    inline void transmitFilteredObstacles(const vector<tObstacleData> &obstacles, const tTimeStamp &time);
};

#endif //TRACKING_FILTER_H
