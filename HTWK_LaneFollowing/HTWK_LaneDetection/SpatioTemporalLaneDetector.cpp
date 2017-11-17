#include "SpatioTemporalLaneDetector.h"

SpatioTemporalLaneDetector::SpatioTemporalLaneDetector(int start, int stop, int step, int frames,
                                                       bool isBottmUp, ThresholdAlgorithm algorithm ) {
    if (isBottmUp) {
        initImagesBottomUp(start, stop, step, frames, algorithm);
    } else {
        initImagesTopDown(start, stop, step, frames, algorithm);
    }
}

/**
 * initalizes the positions of the LaneDetection from highest to lowest
 */
void SpatioTemporalLaneDetector::initImagesBottomUp(int start, int stop, int step, int frames,
                                                    ThresholdAlgorithm algorithm ) {
    if (start < stop) {
        swap(start, stop);
    }
    for (int i = start; i >= stop; i -= step) {
        _spatioImages.push_back(new SpatioTemporalImage(i, frames, algorithm));
    }
}

/**
 * initalizes the positions of the LaneDetection from lowest to highest
 */
void SpatioTemporalLaneDetector::initImagesTopDown(int start, int stop, int step, int frames,
                                                   ThresholdAlgorithm algorithm ) {
    if (start > stop) {
        swap(start, stop);
    }
    for (int i = start; i <= stop; i += step) {
        _spatioImages.push_back(new SpatioTemporalImage(i, frames, algorithm));
    }
}

/**
 * Clears allocated memory
 */
SpatioTemporalLaneDetector::~SpatioTemporalLaneDetector() {
    for(auto it = _spatioImages.begin(); it != _spatioImages.end(); it++){
        delete(*it);
    }
    _spatioImages.clear();
}

/**
 * parallelDetectLanes uses boost to allow multithreaded detection
 */
//TODO Aufwändig::Sollte überarbeitet werden, damit Threads nur einmalig erzeugt werden.... (reduzierung erzeugungs overhead)
std::vector<tLanePoint> SpatioTemporalLaneDetector::parallelDetectLanes(const Mat &input) {
    std::vector<std::thread *> threadVec;
    // if c++ 11 not available use boost::thread
    for (unsigned int imageIndex = 0; imageIndex < _spatioImages.size(); imageIndex++) {
        threadVec.push_back(
                new std::thread(&SpatioTemporalImage::DetectLanesPoints, _spatioImages[imageIndex], input));
    }

    std::vector<tLanePoint> results;
    for (unsigned int index = 0; index < _spatioImages.size(); index++) {
        threadVec[index]->join();
        results.push_back(getPointAsTypeLanePoint(_spatioImages[index]->getDetectionResult()));
        delete threadVec[index];
    }
    threadVec.clear();
    return results;
}

/**
 * laneDetection done sequentiell
 */
std::vector<tLanePoint> SpatioTemporalLaneDetector::detectLanes(const Mat &input) {
    std::vector<tLanePoint> results;
    for (unsigned int index = 0; index < _spatioImages.size(); index++) {
        _spatioImages[index]->DetectLanesPoints(input);
        results.push_back(getPointAsTypeLanePoint(_spatioImages[index]->getDetectionResult()));
    }
    return results;
}

/**
 * converts SpatioLanePointPair to tLanPoints
 */
tLanePoint SpatioTemporalLaneDetector::getPointAsTypeLanePoint(const SpatioLanePointPair &pointPair) {
    tLanePoint resultToTransmit;
    resultToTransmit.xRightLane = pointPair.Right.Point.x;
    resultToTransmit.isRightGuessed = pointPair.Right.Guessed;
    resultToTransmit.xLeftLane = pointPair.Left.Point.x;
    resultToTransmit.isLeftGuessed = pointPair.Left.Guessed;
    resultToTransmit.y = pointPair.Left.Point.y;
    return resultToTransmit;
}
