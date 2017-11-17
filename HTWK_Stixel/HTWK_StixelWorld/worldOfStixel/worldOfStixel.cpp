#include "worldOfStixel.h"

#define MAX_COLOR_VALUE 255
#define SCALAR_GRAY Scalar(100,100,100)

WorldOfStixel::WorldOfStixel(const tCameraProperties &cameraProperties, const int &stixelWidth,
                             const FreeSpaceParameter &freeSpaceParameter,
                             const HeightSegmentationParameter &heightSegmentationParameter)
        : stixelWidth(stixelWidth), maxDisparity(cameraProperties.disparity) {
    imageBasedFreeSpace = unique_ptr<FreeSpace>(new FreeSpace(cameraProperties, freeSpaceParameter));
    heightSegmentation = unique_ptr<HeightSegmentation>(
            new HeightSegmentation(cameraProperties, heightSegmentationParameter));
}

void WorldOfStixel::compute(const Mat &disparityMap) {
    computeFreeSpaceAndHeightSegmentation(disparityMap);
    buildStaticStixel(disparityMap);
}

void WorldOfStixel::computeFreeSpaceAndHeightSegmentation(const Mat &disparityMap) {
    Mat disparityMapTransposed;
    resize(disparityMap, disparityMapTransposed, Size(disparityMap.cols / stixelWidth, disparityMap.rows));
    disparityMapTransposed = disparityMapTransposed.t();
    imageBasedFreeSpace->compute(disparityMapTransposed);
    heightSegmentation->compute(disparityMapTransposed, imageBasedFreeSpace->getComputedLowerPath());
}

void WorldOfStixel::buildStaticStixel(const Mat &disparityMap) {
    vector<int> lowerPath = imageBasedFreeSpace->getComputedLowerPath();
    vector<int> upperPath = heightSegmentation->getComputedUpperPath();

    if (stixels.empty()) {
        tStixel stixel;
        stixels.resize(upperPath.size(), stixel);
    }
#pragma omp parallel for schedule(static, 8) num_threads(4)
    for (unsigned int i = 0; i < lowerPath.size(); i++) {
        int rowTop = 0;
        int rowBottom = 0;
        if (upperPath[i] > lowerPath[i]) {
            rowTop = lowerPath[i];
            rowBottom = upperPath[i];
        } else {
            rowTop = upperPath[i];
            rowBottom = lowerPath[i];
        }

        const Rect stixelArea = Rect(stixelWidth * i, rowTop, stixelWidth, rowBottom - rowTop);

        stixels[i].column = stixelWidth * i + stixelWidth / 2;
        stixels[i].rowTop = rowTop;
        stixels[i].rowBottom = rowBottom;
        stixels[i].disparity = extractDisparity(disparityMap, stixelArea);
    }
}

const float WorldOfStixel::extractDisparity(const Mat &disparityMap, const Rect &stixelArea) {
    const Mat roi(disparityMap, stixelArea & Rect(0, 0, disparityMap.cols, disparityMap.rows));
    const int histSize[] = {maxDisparity + 1};
    const float range[] = {-1, static_cast<float>(maxDisparity)};
    const float *ranges[] = {range};

    Mat hist;
    calcHist(&roi, 1, 0, Mat(), hist, 1, histSize, ranges);

    int maxIdx;
    minMaxIdx(hist, NULL, NULL, NULL, &maxIdx);

    return (range[1] - range[0]) * maxIdx / histSize[0] + range[0];
}

const vector<tStixel> &WorldOfStixel::getComputedStixel() {
    return stixels;
}

const Mat WorldOfStixel::drawStixelWorld(const Mat &leftImage) {
    Mat left;
    if (leftImage.channels() == 1) { //IR-Cam are CV_8UC1
        cvtColor(leftImage, left, CV_GRAY2BGR);
    } else {
        leftImage.copyTo(left);
    }

    Mat stixelImage = Mat::zeros(left.size(), left.type());
#pragma omp parallel for schedule(static, 8) num_threads(4)
    for (unsigned int i = 0; i < stixels.size(); i++) {
        const int radius = max(stixelWidth / 2, 1);
        const Point2f topLeft(stixels[i].column - radius, stixels[i].rowTop);
        const Point2f bottomRight(stixels[i].column + radius, stixels[i].rowBottom);
        rectangle(stixelImage, Rect(topLeft, bottomRight), mapDisparityToColor(stixels[i].disparity), -1);
    }

    return left + stixelImage;
}

const Scalar WorldOfStixel::mapDisparityToColor(const float &disparity) {
    return disparity < 0 ? SCALAR_GRAY : Scalar(0, (maxDisparity - disparity) * MAX_COLOR_VALUE / maxDisparity,
                                                (disparity * MAX_COLOR_VALUE) / maxDisparity);
}
