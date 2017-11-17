#ifndef LANE_DETECTION_FILTER_H
#define LANE_DETECTION_FILTER_H

#include "stdafx.h"
#include "ImageHelper.h"
#include <opencv2/opencv.hpp>
#include "../../HTWK_Types/StixelData.h"
#include "../../HTWK_Types/CameraProperties.h"
#include "../../HTWK_Types/ObstacleData.h"
#include "../../HTWK_Utils/VectorHelper.h"

#define OID_ADTF_FILTER_DEF "htwk.stixels_to_rois"
#define ADTF_FILTER_DESC "HTWK Stixels To ROIs"

#define PROPERTY_FAILURE_DISPARITY "Failure Disparity"
#define PROPERTY_MAX_HEIGHT "Max Height"
#define PROPERTY_MIN_HEIGHT "Min Height"
#define PROPERTY_MAX_HEIGHT_DIFF "Max Height Diff"
#define PROPERTY_MAX_DEPTH_DIFF_BETWEEN_NEIGHBOURS "Max Distance Diff between Neighbourstixels"
#define PROPERTY_MAX_DISTANCE "Max Distance"
#define PROPERTY_MIN_DISTANCE "Min Distance"
#define PROPERTY_MIN_STIXELS_FOR_ROI "Min Stixels ROI"
#define PROPERTY_DEFAULT_MIN_DISTANCE 0.7
#define PROPERTY_DEFAULT_MAX_DISTANCE 3.0
#define PROPERTY_DEFAULT_DEPTH_NEIGHBOURS 0.1

#define LOWER_LIMIT 0.f
#define UPPER_LIMIT_Y 479.0f //Height - 1

class StixelsToRoisFilter : public adtf::cFilter {
ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, adtf::OBJCAT_DataFilter);

public:
    StixelsToRoisFilter(const tChar *__info);

private:
    struct Position {
        Point2f point;
        float radius = 0;
        float speed = 0;
        float heading = 0;
    };

    /*! the id for the f32x of the media description for output pin */
    tBufferID m_szIDPositionF32X;
    /*! the id for the f32y of the media description for output pin */
    tBufferID m_szIDPositionF32Y;
    /*! the id for the af32radius of the media description for output pin */
    tBufferID m_szIDPositionF32Radius;
    /*! the id for the af32speed of the media description for output pin */
    tBufferID m_szIDPositionF32Speed;
    /*! the id for the af32heading of the media description for output pin */
    tBufferID m_szIDPositionF32Heading;

    tBool positionIDsSet = false;

    Position pos;

    cInputPin inputPinCameraProperty;

    cInputPin stixelsInputPin;

    cVideoPin rgbInputPin;

    cVideoPin rgbOutputPin;

    bool imageUpdated = false;

    bool stixelsUpdated = false;

    cVideoPin rgbDebugOutputPin;

    cOutputPin obstacleWorldMapOutputPin;

    cInputPin positionPin;

    cObjectPtr<IMediaTypeDescription> positionDescription;

    tResult Start(ucom::IException **__exception_ptr);

    tResult Init(tInitStage eStage, ucom::IException **__exception_ptr);

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

    inline void initPropertys();

    inline void extractRois();

    inline void debugDrawRoisRGB();

    inline tResult transmitObstacles();

    inline tResult transmitVideo(const Mat &image, cVideoPin &pin, tBitmapFormat &bmp);

    inline bool
    checkForBreak(const tStixel &check, const tStixel &stop, const Point2f &topLeft, const Point2f &bottomRight,
                  const float &disp);

    inline Point2f transferIrPointToRGB(const Point2f &fromPoint, const float &disparity);

    template<typename Type>
    inline Type checkLimits(const Type &lower, const Type &value, const Type &upper) {
        return min(max(lower, value), upper);
    }

    tBitmapFormat rgbInputBitmapFormat;

    tBitmapFormat rgbOutputBitmapFormat;

    tBitmapFormat debugOutputBitmapFormat;

    Mat rgbInputImage;

    vector<tStixel> tStixelVector;

    vector<tObstacleData> obstacleWorld;

    tCameraProperties cProp;

    bool isAlreadyInitalized = false;

    int failureDisp = -1; // SSGBM Algorithm

    float maxHeight = 0.4;

    float minHeight = 0.09;

    float maxHeightDiff = 0.1;

    float maxDisparityDiffBetweenNeighbours = 10;

    float minDisparity = 7; // Large Disparity -> close, Small -> far away

    float maxDisparity = 63;

    unsigned int minStixelsForRoi = 3; //StixelWidth dependend!

    int stixelWidthOffset = 1;

    float upperLimit = 0;

    bool cPropUpdated = false;

    tTimeStamp stixelsTimestamp = 0;

    unsigned int ignoreLeftOffset = 0;

    tUInt64 dataID = 0;

    const Point2f infraredToImuTranslation = Point2f(0.04, 0.363);

    inline void initalizeMemberVariables(const vector<tStixel> &stixels);

    inline const Point3f getPoint3D(const Point2f &pt, const float disp) const;
};

#endif //LANE_DETECTION_FILTER_H
