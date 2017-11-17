#ifndef PROJECT_VISIONUTILS_H
#define PROJECT_VISIONUTILS_H

#include <opencv2/opencv.hpp>
#include "stdafx.h"

class HTWKVisionUtils {
public:
    static const int Yen(const cv::Mat &image);

    inline static const float calcCrit(const float &P1, const float &P1_sq, const float &P2_sq);

    static const int GetMatType(tInt16 bitsPerPixel);

    static const cv::Mat ExtractImageFromMediaSample(IMediaSample *mediaSample, const tBitmapFormat &inputFormat);
};

#endif //PROJECT_VISIONUTILS_H
