#ifndef PROJECT_SPATIOTEMPORALIMAGE_H
#define PROJECT_SPATIOTEMPORALIMAGE_H

#include <opencv2/opencv.hpp>
#include "../../HTWK_Utils/HTWKUtils.hpp"
#include "../../HTWK_Debug/EnableLogs.h"
#define ADAPTIVE_METHODE_OFFSET 2

using namespace cv;

enum ThresholdAlgorithm {
    YEN = CV_THRESH_BINARY,
    YEN_OTSU = CV_THRESH_BINARY | CV_THRESH_OTSU,
    YEN_TRIANGLE = CV_THRESH_BINARY | CV_THRESH_TRIANGLE,
    ADAPTIVE_MEAN_C = CV_ADAPTIVE_THRESH_MEAN_C - ADAPTIVE_METHODE_OFFSET,
    ADAPTIVE_GAUSSIAN_C = CV_ADAPTIVE_THRESH_GAUSSIAN_C - ADAPTIVE_METHODE_OFFSET
};

struct SpatioLanePoint {
    cv::Point2f Point;
    bool Guessed = true;
};

struct SpatioLanePointPair {
    SpatioLanePoint Left;
    SpatioLanePoint Right;
};

// Used for morphological operations
const static Mat structuringElement = cv::getStructuringElement(CV_SHAPE_ELLIPSE, Size(3, 3));

class SpatioTemporalImage {
public:
    SpatioTemporalImage(const int &scanLineHeight, const int &frameCount, const ThresholdAlgorithm &algorithm = YEN);

    void DetectLanesPoints(const Mat &input);

    const SpatioLanePointPair &getDetectionResult();

private:
    int _scanLineHeight;
    int _frameCount;
    ThresholdAlgorithm algorithm;

    Mat _spatioTemporalImage;
    Mat _lastScanLine;
    int _lastInsertionX;

    SpatioLanePoint _lastLeftPoint;
    SpatioLanePoint _lastRightPoint;

    SpatioLanePointPair detectionResult;

    const int FindDisplacement(const Mat &scanLine, const Mat &lastScanLine);

    inline const bool IsUnique(const Vec2f &currentLine, const std::vector<Vec2f> &strongLines);

    inline const float FindIntersectionWithScanline(const Vec2f &houghLine);

    const double calculateDifferenceInRoi(const int &range, const Point2f &referencePoint,
                                          const Mat &extendedLastScanLine, const Mat &extendedScanLine);

    const SpatioLanePoint FindLanePoint(const Vec2f &houghLine, const Mat &binaryScanLine, const bool &leftEnd,
                                        const int &displacement, SpatioLanePoint fallBack);

    void initSpatioTempImage(const Mat &scanLine);

    void findBestLeftAndRightLane(const Mat &scanLine, const int &insertionX, const std::vector<Vec2f> &strongLines,
                                  Vec2f &leftHoughLine, Vec2f &rightHoughLine);

    inline void filterSpatioImage(Mat &temporaryStorage);

    std::vector<Vec2f> findAndFilterHoughLines(const Mat &temporaryStorage);

    void calcLanePointsAndCorrectError(const Mat &temporaryStorage,
                                       const Vec2f &leftHoughLine, const Vec2f &rightHoughLine);
};

#endif //PROJECT_SPATIOTEMPORALIMAGE_H
