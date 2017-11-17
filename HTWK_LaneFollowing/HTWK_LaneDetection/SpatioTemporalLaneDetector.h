#ifndef PROJECT_SPATIOTEMPORALLANEDETECTOR_H
#define PROJECT_SPATIOTEMPORALLANEDETECTOR_H

//#include <boost/thread.hpp>
#include <thread> //C++ 11 necessary! Else use boost
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include "SpatioTemporalImage.h"
#include "../../HTWK_Types/LaneData.h"
#include "../../HTWK_Debug/EnableLogs.h"
using namespace cv;

class SpatioTemporalLaneDetector {
private:
    std::vector<SpatioTemporalImage*> _spatioImages;

    void initImagesBottomUp(int start, int stop, int step, int frames, ThresholdAlgorithm algorithm);

    void initImagesTopDown(int start, int stop, int step, int frames, ThresholdAlgorithm algorithm);

    tLanePoint getPointAsTypeLanePoint(const SpatioLanePointPair &pointPair);

public:
    SpatioTemporalLaneDetector(int start, int stop, int step, int frames,
                               bool isBottumUp = true, ThresholdAlgorithm algorithm = YEN);

    ~SpatioTemporalLaneDetector();

    std::vector<tLanePoint> parallelDetectLanes(const Mat &input);

    std::vector<tLanePoint> detectLanes(const Mat &input);

};

#endif //PROJECT_SPATIOTEMPORALLANEDETECTOR_H
