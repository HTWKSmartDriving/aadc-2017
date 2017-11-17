#ifndef PROJECT_IMAGEBASEDFREESACECOMPUTATION_H
#define PROJECT_IMAGEBASEDFREESACECOMPUTATION_H

#include <opencv2/opencv.hpp>
#include "../../../../HTWK_Types/CameraProperties.h"

#define FREE_SPACE_P1 50
#define FREE_SPACE_P2 32
#define FREE_SPACE_MAX_PIXEL_JUMP 100
#define FREE_SPACE_OBJECT_HEIGHT 0.1

struct FreeSpaceParameter {
    int parameterOne = FREE_SPACE_P1;
    int parameterTwo = FREE_SPACE_P2;
    int maxPixelJump = FREE_SPACE_MAX_PIXEL_JUMP;
    float objectHeight = FREE_SPACE_OBJECT_HEIGHT;
};

using namespace cv;
using namespace std;

class FreeSpace {

public:
    FreeSpace(const tCameraProperties &cameraProperties, const FreeSpaceParameter &freeSpaceParameter);

    void compute(const Mat1f &disparityMapTransposed);

    const vector<int> &getComputedLowerPath();

private:
    inline void createRoadDisparityVector(const int &vMax);

    inline void initScore(const Mat1f &disparityMapTransposed, Mat1f &score);

    inline void initVTop(const int &vMax);

    inline void calcScore(const Mat1f &disparityMapTransposed, Mat1f &score);

    inline void extractLowerPath(const Mat1f &disparityMapTransposed, Mat1f &score);

    vector<float> roadDisparity;

    vector<int> vTopology;

    int vHorizon;

    tCameraProperties cProp;

    vector<int> lowerPath;

    const FreeSpaceParameter freeSpaceParam;

    const float SCORE_PENALITY_ROAD = -1.f;

    const float SCORE_DEFAULT = 1.f;

    const int PARAMETER_ROAD = 1;

    const int PARAMETER_OBJECT = 2;
};

#endif //PROJECT_IMAGEBASEDFREESACECOMPUTATION_H
