#ifndef PROJECT_HEIGHTSEGMENTATION_H
#define PROJECT_HEIGHTSEGMENTATION_H

#include "opencv2/opencv.hpp"
#include "../../../../HTWK_Types/CameraProperties.h"

#define HEIGHT_SEG_C_S 8
#define HEIGHT_SEG_N_Z 0.1
#define HEIGHT_SEG_MAX_PIXEL_JUMP  50
#define HEIGHT_SEG_DELTA_Z 0.5

struct HeightSegmentationParameter {
    float deltaZ = HEIGHT_SEG_DELTA_Z;
    float N_z = HEIGHT_SEG_N_Z; //Neighbours in Z Direction (in Meter)
    int C_s = HEIGHT_SEG_C_S; //Cost Smoothness Penalty
    int maxPixelJump = HEIGHT_SEG_MAX_PIXEL_JUMP;
};

using namespace cv;
using namespace std;

class HeightSegmentation {

public:
    HeightSegmentation(const tCameraProperties &cameraProperties,
                       const HeightSegmentationParameter &segmentationParameter);

    void compute(const Mat1f &disparityMapTransposed, const vector<int> &lowerPath);

    const vector<int> &getComputedUpperPath();

private:
    inline void calcScore(const Mat1f &disparityMapTransposed, const vector<int> &lowerPath, Mat1f &score);

    inline void extractUpperPath(const Mat1f &disparityMapTransposed, const vector<int> &lowerPath, Mat1f &score);

    vector<int> upperPath;

    const HeightSegmentationParameter segParameter;

    const float zDepthFactor;
};
#endif //PROJECT_HEIGHTSEGMENTATION_H
