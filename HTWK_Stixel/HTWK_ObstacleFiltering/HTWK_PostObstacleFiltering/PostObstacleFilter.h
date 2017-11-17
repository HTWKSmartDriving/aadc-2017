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
#include "../../../HTWK_Types/TrackingData.h"
#include "../../../HTWK_Utils/VectorHelper.h"
#include "../../../HTWK_Types/InitalRoadSign.h"
#include "../../../HTWK_Debug/EnableLogs.h"

#define PROPERTY_MIN_DIST "Minimum Distance Merging Objects"
#define PROPERTY_DEFAULT_MIN_DIST 0.35f
#define PROPERTY_INTERVALL_CLEAN_UP_TIME "CleanUp Check Time"
#define PROPERTY_DEFAULT_INTERVALL_CLEAN_UP_TIME 1.0
#define PROPERTY_TRACKED_TIME_OUT "Timeout tracked obstacles"
#define PROPERTY_DEFAULT_TRACKED_TIME_OUT 10.0
#define PROPERTY_MOVEMENT_COUNT "Minimum Count for Movement"
#define PROPERTY_DEFAULT_MOVEMENT_COUNT 3
#define PROPERTY_THRESHOLD_BEST_SCORE "Threshold Best Score"
#define PROPERTY_DEFAULT_THRESHOLD_BEST_SCORE 0.8

#define OID_ADTF_FILTER_DEF "htwk.postObstacleFilter"
#define ADTF_FILTER_DESC "HTWK Post Obstacle Filter"

class PostObstacleFilter : public adtf::cFilter {
ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, adtf::OBJCAT_DataFilter);

public:
    PostObstacleFilter(const tChar *__info);

private:
    struct ObstacleTrackingData {
        tTrackingData trackingData;
        long countForMovement = 0;
        tTimeStamp ticks = 0;
    };

    cObjectPtr<IMediaTypeDescription> m_pDescNewObstacleJury;
    struct NewObstacleIDS {
        tBufferID m_szIDf32X;
        tBufferID m_szIDf32Y;
        tBool newObstacleIDsSet = false;
    } newObstacleIDs;

    tResult Init(tInitStage eStage, ucom::IException **__exception_ptr);

    tResult Start(ucom::IException **__exception_ptr);

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

    inline tResult transmitVideo(const Mat &image, cVideoPin &pin, tBitmapFormat &bmp);

    inline void transmitTrackedObstacles(const vector<tTrackingData> &trackedObstacles, const tTimeStamp &time);

    inline tTimeStamp GetTime() { return adtf_util::cHighResTimer::GetTime(); }

    inline void removeOutdatedObstacles();

    inline void trackObstacles(const list<ObstacleTrackingData> &matchedSamples, const tTimeStamp &time);

    inline bool checkMovableType(const ObstacleType &type) {
        return (type == ObstacleType::CAR || type == ObstacleType::ADULT || type == ObstacleType::CHILD);
    };

    inline bool checkTimeOut(const tTimeStamp &current, const tTimeStamp &previous, const float &timeOutTime) {
        return (current - previous) * 1e-6 > timeOutTime;
    };

    inline bool
    findMatchingObstacles(vector<tObstacleData> &inputSamples, list<ObstacleTrackingData> &outputMatchedSamples);

    inline void updateStaticObstaclePosition(const ObstacleTrackingData &match,
                                             const list<PostObstacleFilter::ObstacleTrackingData>::iterator &obst);

    inline void
    updateDynamicObstaclePosition(const ObstacleTrackingData &match,
                                      list<PostObstacleFilter::ObstacleTrackingData>::iterator &obst);

    inline void
    checkAndUpdateOverlap(const ObstacleTrackingData &match,
                              list<PostObstacleFilter::ObstacleTrackingData>::iterator &obst);

    inline tResult transmitNewObstaclesToJury(const tTrackingData &newObstacle, const tTimeStamp &time);

#ifdef DEBUG_POST_OBSTACLE_LOG
    Mat debugWorldMap;
#endif

    cVideoPin videoDebugOutputPin;

    cInputPin obstacleInputPin;

    cOutputPin trackedOutputPin;

    cOutputPin newObstacleJuryPin;

    tBitmapFormat videoOutputBitmapFormat;

    list<ObstacleTrackingData> trackedObstacles;

    vector<tObstacleData> prevInputSamples;

    tTimeStamp currentTime = 0;
    float timeOutTrackedObstacles = PROPERTY_DEFAULT_TRACKED_TIME_OUT;
    float intervallCheckTime = PROPERTY_DEFAULT_INTERVALL_CLEAN_UP_TIME;
    float minimumMergeDistance = PROPERTY_DEFAULT_MIN_DIST;
    int countAsMovement = PROPERTY_DEFAULT_MOVEMENT_COUNT;

    float thresholdBestScore = PROPERTY_DEFAULT_THRESHOLD_BEST_SCORE;

    inline void rearCamHandling(const vector<tObstacleData> &inputSamples);
};

#endif //TRACKING_FILTER_H
