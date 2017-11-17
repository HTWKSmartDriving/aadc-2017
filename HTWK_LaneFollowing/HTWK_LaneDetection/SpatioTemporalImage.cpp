#include "HTWKVisionUtils.h"
#include "SpatioTemporalImage.h"
#include <adtf_plugin_sdk.h>

SpatioTemporalImage::SpatioTemporalImage(const int &scanLineHeight, const int &frameCount,
                                         const ThresholdAlgorithm &selectedAlgorithm) {
    _scanLineHeight = scanLineHeight;
    _frameCount = frameCount;
    algorithm = selectedAlgorithm;
    //inital values of detectionResult
    detectionResult.Left.Point = Point2f(0, 0);
    detectionResult.Left.Guessed = true;
    detectionResult.Right = detectionResult.Left;
}

void SpatioTemporalImage::DetectLanesPoints(const Mat &input) {
    // Extract ScanLine
    Mat scanLine;
    cvtColor(input.row(_scanLineHeight), scanLine, CV_BGR2GRAY);

    // Initialize on first frame
    if (_spatioTemporalImage.empty()) {
        initSpatioTempImage(scanLine);
    }

    int spatioInsertionX = 0;
    int insertionX = _lastInsertionX + FindDisplacement(scanLine, _lastScanLine);
    int colsForTemporaryStorage = _spatioTemporalImage.cols;

    // Correct InsertionX and initialize temporary storage
    if (insertionX < 0) {
        // Extend to the left
        colsForTemporaryStorage = _spatioTemporalImage.cols - insertionX;
        spatioInsertionX = -insertionX;
        insertionX = 0;
        //temporaryStorage muss mindestens so groß sein wie scanline.cols!!
        //TODO Überprüfen im realen Fahrumfeld
        //-----------------------------------------------------
        if (colsForTemporaryStorage < scanLine.cols) {
            int diff = scanLine.cols - colsForTemporaryStorage;
            colsForTemporaryStorage = diff + colsForTemporaryStorage;
        }
        //-----------------------------------------------------
    } else if (scanLine.cols + insertionX > _spatioTemporalImage.cols) {
        // Extend to the right
        colsForTemporaryStorage = scanLine.cols + insertionX;
    }// else :Leave it as it is

    Mat temporaryStorage = Mat::zeros(_frameCount, colsForTemporaryStorage, CV_8UC1);
    // Add new scanline and remove last
    _spatioTemporalImage(Rect(0, 0, _spatioTemporalImage.cols, _spatioTemporalImage.rows - 1))
            .copyTo(temporaryStorage(
                    Rect(spatioInsertionX, 1, _spatioTemporalImage.cols, _spatioTemporalImage.rows - 1)));

    try {
        scanLine.copyTo(temporaryStorage(Rect(insertionX, 0, scanLine.cols, 1)));
    }
    catch (cv::Exception &e) {
#ifdef DEBUG_SPATIO_TEMPORAL_IMAGE_LOG
        LOG_ERROR(adtf_util::cString::Format("ScanLineCopy failed: %s", e.what()));
        //    LOG_ERROR(adtf_util::cString::Format("ScanLine %d %d", _scanLineHeight));
        //    LOG_ERROR(adtf_util::cString::Format("InsertionX: %d, LastInsert %d", insertionX, _lastInsertionX));
#endif
    }

    Rect boundingRect;

    //Falls mehr Schwarz, als weiße Pixel, dann schneide nicht zu
    //TODO Überprüfen im realen Fahrumfeld
    //-----------------------------------------------------
    int notBlackPixels = countNonZero(temporaryStorage);
    int totalPixels = temporaryStorage.rows * temporaryStorage.cols;
    int zeroPixels = totalPixels - notBlackPixels;
    if (zeroPixels > notBlackPixels) {
        boundingRect = Rect(0, 0, _spatioTemporalImage.cols, 1);
    }
        //------------------------------------------------------
    else {
        Mat Points;
        findNonZero(temporaryStorage, Points);
        boundingRect = cv::boundingRect(Points);
    }

    try {
        _spatioTemporalImage = temporaryStorage(Rect(boundingRect.x, 0, boundingRect.width, _frameCount)).clone();
    }
    catch (cv::Exception &e) {
#ifdef DEBUG_SPATIO_TEMPORAL_IMAGE_LOG
        LOG_ERROR(adtf_util::cString::Format("BoundingRectFailed: %s", e.what()));
        //LOG_ERROR(adtf_util::cString::Format("Rect x: %d, widht: %d", boundingRect.x, boundingRect.width));
#endif
    }

    //Correct Coordinate-System
    _lastScanLine = scanLine;
    _lastInsertionX = insertionX - boundingRect.x;
    _lastLeftPoint.Point.x += spatioInsertionX - boundingRect.x;
    _lastRightPoint.Point.x += spatioInsertionX - boundingRect.x;

    filterSpatioImage(temporaryStorage);
    std::vector<Vec2f> strongLines = findAndFilterHoughLines(temporaryStorage);

    Vec2f leftHoughLine, rightHoughLine;
    findBestLeftAndRightLane(scanLine, insertionX, strongLines, leftHoughLine, rightHoughLine);
    calcLanePointsAndCorrectError(temporaryStorage, leftHoughLine, rightHoughLine);
}

void SpatioTemporalImage::initSpatioTempImage(const Mat &scanLine) {

    _spatioTemporalImage = Mat::zeros(_frameCount, scanLine.cols, CV_8UC1);
    _lastScanLine = Mat::zeros(1, scanLine.cols, CV_8UC1);
    _lastInsertionX = 0;

    _lastLeftPoint.Point = Point2f(0, _scanLineHeight);
    _lastLeftPoint.Guessed = true;
    _lastRightPoint.Point = Point2f(scanLine.cols - 1, _scanLineHeight);
    _lastRightPoint.Guessed = true;
}

const int SpatioTemporalImage::FindDisplacement(const Mat &scanLine, const Mat &lastScanLine) {

    if (sum(scanLine)[0] < 1000 || sum(lastScanLine)[0] < 1000) {
        return 0;
    }

    const int maximalAdditionalDisplacement = 10;
    const int comparisonWidth = lastScanLine.cols + 2 * maximalAdditionalDisplacement;

    int bestDisplacement = -maximalAdditionalDisplacement;
    double minimalDifference = INT_MAX;

    Mat extendedLastScanLine = Mat::zeros(1, comparisonWidth, CV_8UC1);
    lastScanLine.copyTo(extendedLastScanLine(Rect(maximalAdditionalDisplacement, 0, lastScanLine.cols, 1)));
    Mat extendedScanLine = Mat::zeros(1, comparisonWidth, CV_8UC1);

    for (int displacement = -maximalAdditionalDisplacement;
         displacement < maximalAdditionalDisplacement; ++displacement) {

        scanLine.copyTo(extendedScanLine(Rect(displacement + maximalAdditionalDisplacement, 0, scanLine.cols, 1)));
        if (_lastRightPoint.Guessed && _lastLeftPoint.Guessed) {
            return 0;
        }

        double currentDifference = 0;
        if (!_lastLeftPoint.Guessed) {
            currentDifference += calculateDifferenceInRoi(50,
                                                          _lastLeftPoint.Point +
                                                          Point2f(maximalAdditionalDisplacement, 0),
                                                          extendedLastScanLine,
                                                          extendedScanLine);
        }

        if (!_lastRightPoint.Guessed) {
            currentDifference += calculateDifferenceInRoi(50,
                                                          _lastRightPoint.Point +
                                                          Point2f(maximalAdditionalDisplacement, 0),
                                                          extendedLastScanLine,
                                                          extendedScanLine);
        }

        if (currentDifference < minimalDifference) {
            bestDisplacement = displacement;
            minimalDifference = currentDifference;
        }
    }

    return bestDisplacement;
}

const double SpatioTemporalImage::calculateDifferenceInRoi(const int &range, const Point2f &referencePoint,
                                                           const Mat &extendedLastScanLine,
                                                           const Mat &extendedScanLine) {

    float roiStartX = referencePoint.x - range / 2;

    if (roiStartX < 0) {
        roiStartX = 0;
    } else if (roiStartX + range > extendedScanLine.cols) {
        roiStartX = extendedScanLine.cols - range - 1;
    }

    Rect2f roi(roiStartX, 0, range, 1);
    Mat difference;
    absdiff(extendedScanLine(roi), extendedLastScanLine(roi), difference);

    return sum(difference)[0];
}

inline void SpatioTemporalImage::filterSpatioImage(Mat &temporaryStorage) {
    medianBlur(_spatioTemporalImage, temporaryStorage, 5);

    if (algorithm < 0) {
        adaptiveThreshold(temporaryStorage, temporaryStorage, 255, algorithm + ADAPTIVE_METHODE_OFFSET,
                          CV_THRESH_BINARY, 3, 0);

    } else {
        threshold(temporaryStorage, temporaryStorage, HTWKVisionUtils::Yen(temporaryStorage), 255, algorithm);
    }

    morphologyEx(temporaryStorage, temporaryStorage, MORPH_OPEN, structuringElement);
    morphologyEx(temporaryStorage, temporaryStorage, MORPH_CLOSE, structuringElement);
}

std::vector<Vec2f> SpatioTemporalImage::findAndFilterHoughLines(const Mat &temporaryStorage) {
    // Find dominant lines
    std::vector<Vec2f> lines;
    HoughLines(temporaryStorage, lines, 10, 90 * CV_PI / 180, 5, 0);

    // Filter dominant lines
    std::vector<Vec2f> strongLines;
    for (unsigned int lineIndex = 0; lineIndex < lines.size(); ++lineIndex) {
        if (strongLines.size() >= 10) {
            break;
        }

        Vec2f currentLine = lines[lineIndex];

        if (currentLine[1] > (CV_PI / 18) && currentLine[1] < (CV_PI - (CV_PI / 18))) {
            continue;
        }

        if (IsUnique(currentLine, strongLines)) {
            strongLines.push_back(currentLine);
        }
    }
    return strongLines;
}

inline const bool SpatioTemporalImage::IsUnique(const Vec2f &currentLine, const std::vector<Vec2f> &strongLines) {
    for (unsigned int strongLineIndex = 0; strongLineIndex < strongLines.size(); ++strongLineIndex) {
        if (abs(currentLine[0] - strongLines[strongLineIndex][0]) < 50) {
            return false;
        }
    }
    return true;
}

void SpatioTemporalImage::findBestLeftAndRightLane(const Mat &scanLine, const int &insertionX,
                                                   const std::vector<Vec2f> &strongLines,
                                                   Vec2f &leftHoughLine, Vec2f &rightHoughLine) {

    Vec2f currentLine;
    float x, bestX;
    bool isLeft;
    for (unsigned int lineIndex = 0; lineIndex < strongLines.size(); ++lineIndex) {
        currentLine = strongLines[lineIndex];
        x = FindIntersectionWithScanline(currentLine);

        if (x < (scanLine.cols / 2.f + insertionX)) {
            //Left
            bestX = FindIntersectionWithScanline(leftHoughLine);
            isLeft = true;
        } else {
            //Right
            bestX = FindIntersectionWithScanline(rightHoughLine);
            isLeft = false;
        }
        if (abs(x - (scanLine.cols / 2.f + insertionX)) < abs(bestX - (scanLine.cols / 2.f + insertionX))) {
            isLeft == true ? leftHoughLine = currentLine : rightHoughLine = currentLine;
        }
    }

}

inline const float SpatioTemporalImage::FindIntersectionWithScanline(const Vec2f &houghLine) {
    //rho / cosf(theta)
    return (houghLine[0] / cosf(houghLine[1]));
}

const SpatioLanePoint SpatioTemporalImage::FindLanePoint(const Vec2f &houghLine,
                                                         const Mat &binaryScanLine,
                                                         const bool &leftEnd,
                                                         const int &displacement,
                                                         SpatioLanePoint fallBack) {
    fallBack.Guessed = true;
    const float intersection = FindIntersectionWithScanline(houghLine);

    std::vector<float> whiteCenterPixels;
    bool whiteEncountered = false;
    int whiteRange = 0;

    for (int pixel = 0; pixel < binaryScanLine.cols; ++pixel) {
        if (binaryScanLine.at<uchar>(Point2f(pixel, 0)) == 0) {
            // Black
            if (!whiteEncountered) {
                continue;
            }

            whiteEncountered = false;
            if (leftEnd) {
                whiteCenterPixels.push_back(pixel - whiteRange);
            } else {
                whiteCenterPixels.push_back(pixel - 0);
            }
            whiteRange = 0;
        } else {
            // White
            whiteEncountered = true;
            whiteRange++;
        }
    }

    if (whiteEncountered) {
        if (leftEnd) {
            whiteCenterPixels.push_back(binaryScanLine.cols - 1 - whiteRange);
        } else {
            whiteCenterPixels.push_back(binaryScanLine.cols - 1);
        }
    }

    if (whiteCenterPixels.empty()) {
        return fallBack;
    }

    std::pair<float, float> nearestCenters = HTWKUtils::GetNearestElements(intersection, whiteCenterPixels);

    float nearestCenter = nearestCenters.first;
    if (abs(nearestCenters.first - intersection) > abs(nearestCenters.second - intersection)) {
        nearestCenter = nearestCenters.second;
    }

    if (abs(nearestCenter - intersection) > 30) {
        return fallBack;
    }

    SpatioLanePoint result;
    result.Point = Point2f(nearestCenter, _scanLineHeight) - Point2f(displacement, 0);
    result.Guessed = false;

    return result;
}

void SpatioTemporalImage::calcLanePointsAndCorrectError(const Mat &temporaryStorage, const Vec2f &leftHoughLine,
                                                        const Vec2f &rightHoughLine) {
    // Calculate LanePoints
    SpatioLanePointPair calcResult;
    const Mat binaryScanLine = temporaryStorage.row(0);
    calcResult.Left = FindLanePoint(leftHoughLine, binaryScanLine, false, _lastInsertionX, _lastLeftPoint);
    calcResult.Right = FindLanePoint(rightHoughLine, binaryScanLine, true, _lastInsertionX, _lastRightPoint);

    // Correct Errors
    if (calcResult.Right.Point.x < calcResult.Left.Point.x) {
        calcResult.Left = _lastLeftPoint;
        calcResult.Left.Guessed = _lastLeftPoint.Guessed;

        calcResult.Right = _lastRightPoint;
        calcResult.Right.Guessed = _lastLeftPoint.Guessed;

    } else if (abs(calcResult.Right.Point.x - calcResult.Left.Point.x) < 20) {
        // Points are too near
        if (abs(calcResult.Left.Point.x - _lastLeftPoint.Point.x) >
            abs(calcResult.Right.Point.x - _lastRightPoint.Point.x)) {
            // Left jumped
            calcResult.Left = _lastLeftPoint;
            calcResult.Left.Guessed = true;
        } else {
            // Right jumped
            calcResult.Right = _lastRightPoint;
            calcResult.Right.Guessed = true;
        }
    }

    _lastLeftPoint = calcResult.Left;
    _lastRightPoint = calcResult.Right;
    detectionResult = calcResult;

}

const SpatioLanePointPair &SpatioTemporalImage::getDetectionResult() {
    return detectionResult;
}
