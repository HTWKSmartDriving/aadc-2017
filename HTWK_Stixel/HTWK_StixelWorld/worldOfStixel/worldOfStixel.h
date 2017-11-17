#ifndef PROJECT_STIXEL_H
#define PROJECT_STIXEL_H

#include <opencv2/opencv.hpp>
#include "../../../HTWK_Types/CameraProperties.h"
#include "imageBasedFreeSpaceComputation/imageBasedFreeSpaceComputation.h"
#include "heightSegmentation/heightSegmentation.h"
#include "../../../HTWK_Types/StixelData.h"

using namespace cv;
using namespace std;

class WorldOfStixel {

public:
    WorldOfStixel(const tCameraProperties &camProperties, const int &stixelWidth,
                  const FreeSpaceParameter &freeSpaceParameter,
                  const HeightSegmentationParameter &heightSegmentationParameter);

    void compute(const Mat &disparityMap);

    const vector<tStixel> &getComputedStixel();

    const Mat drawStixelWorld(const Mat &leftImage);

private:
    inline void buildStaticStixel(const Mat &disparityMap);

    const inline float extractDisparity(const Mat &disparityMap, const Rect &stixelArea);

    inline void computeFreeSpaceAndHeightSegmentation(const Mat &disparityMap);

    const inline Scalar mapDisparityToColor(const float &disparity);

    unique_ptr<HeightSegmentation> heightSegmentation;

    unique_ptr<FreeSpace> imageBasedFreeSpace;

    const int stixelWidth;

    const int maxDisparity;

    vector<tStixel> stixels;
};
#endif //PROJECT_STIXEL_H
