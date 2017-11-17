#ifndef LIGHT_DETECTION_FILTER_H
#define LIGHT_DETECTION_FILTER_H

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
#include "../HTWK_Types/ObstacleData.h"

#define PROPERTY_FLIP_VERTICAL "Flip Vertical"
#define PROPERTY_FLIP_HORIZONTAL "Flip Horizontal"

#define OID_ADTF_FILTER_DEF "htwk.light_detection"
#define ADTF_FILTER_DESC "HTWK Light Detection"

class LightDetector : public adtf::cFilter , IThreadFunc{
ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, adtf::OBJCAT_Auxiliary);

public:
    LightDetector(const tChar *__info);

private:
    cVideoPin videoInputPin;

    cVideoPin videoDebugPin;

    cVideoPin videoOriginalPin;

    cOutputPin obstacleOutputPin;

    tBitmapFormat debugBitmapFormat;

    tBitmapFormat outputOriginalFormat;

    tBitmapFormat inputBitmapFormat;

    tResult transmitVideo(Mat &image, cVideoPin &pin, tBitmapFormat &format);

    tResult Start(ucom::IException **__exception_ptr);

    tResult Stop(ucom::IException **__exception_ptr);

    tResult Shutdown(tInitStage eStage, __exception);

    tResult Init(tInitStage eStage, ucom::IException **__exception_ptr);

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

    inline void transmitObstacles(const vector<tObstacleData> &obstacles);

    inline void extractPossibleCarRects(const Mat &binaryBlueChannel);

    inline void tryToFilterCars();

    inline void findLight();

    inline void initalize();

    tTimeStamp time = 0;

    Mat inputImage;

    Mat croppedImage;

    bool init = false;

    SimpleBlobDetector::Params params;

    Ptr<SimpleBlobDetector> detector;

    unsigned long id = 0;

    int flipper = INT_MAX;

    const int includeRoiBorder = 1; //in px

    const int minAreaSizeCar = 400;

    vector<Rect> possibleBoundRect;

    vector<KeyPoint> lightDetectionResult;

    vector<Rect> resultRect;

    const Rect crop = Rect(50, 110, 400, 200);

    const Size defaultSize = Size(640,480);

    const Mat structuringElement = cv::getStructuringElement(MorphShapes::MORPH_RECT, Point(3, 3));

    cThread thread;

    tResult ThreadFunc(cThread *Thread, tVoid *data, tSize size);
};

#endif //LIGHT_DETECTION_FILTER_H
