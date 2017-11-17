#include "LaneSteeringCalculator.h"

LaneSteeringCalculator::LaneSteeringCalculator(const int frameWidth, const int frameHeight) {
    //frameCenter needed for calculation
    pictureCenter = HTWKPoint(frameWidth / HALF, frameHeight / HALF);
}

/**
 * Berechnet anhand der gefundenen Punkte den Abstand der Spur vom Bildmittelpunkt
 **/
void LaneSteeringCalculator::calcSteering(const std::vector<tLanePoint> &tLanePointVector) {
    iteratePoints(tLanePointVector);
    calcSteeringAngle();
    resetLanePoints();
}

/**
 * Updated die Punkte aus dem Array
 **/
void LaneSteeringCalculator::iteratePoints(const std::vector<tLanePoint> &tLanePointVector) {
    //C98 std::vector<tLanePoint>::const_iterator it = tLanePointVector.begin();
    for (auto it = tLanePointVector.begin(); it != tLanePointVector.end(); it++) {
        updateLanePoints(*it);
    }

}

/**
* Speichert betrachtet Punkte, damit diese für die Winkelberechnung genutzt werden
**/
void LaneSteeringCalculator::updateLanePoints(const tLanePoint &point) {
    this->leftLane.emplace_back(HTWKPoint((point.xLeftLane), point.y));
    this->rightLane.emplace_back(HTWKPoint(point.xRightLane, point.y));
}

/**
* Berechnetz zu setzenden Winkel anhand der Verschiebung der Spuren im Bezug zum Bildmittelpunkt
**/
void LaneSteeringCalculator::calcSteeringAngle() {
    std::sort(this->leftLane.begin(), this->leftLane.end(), HTWKPoint::XGreater);
    std::sort(this->rightLane.begin(), this->rightLane.end(), HTWKPoint::XGreater);

    //kill bad points
    leftLane.pop_back();

    size_t sizeLeft = leftLane.size();
    size_t sizeRight = rightLane.size();

    double medianLeft = 0;
    double medianRight = 0;

    //calculate median values of lane point vectors
    if (sizeLeft  % 2 == 0) {
        medianLeft = (leftLane[sizeLeft / 2 - 1].x() + leftLane[sizeLeft / 2].x()) / 2;
    } else {
        medianLeft = leftLane[sizeLeft / 2].x();
    }

    if (sizeRight  % 2 == 0) {
        medianRight = (rightLane[sizeRight / 2 - 1].x() + rightLane[sizeRight / 2].x()) / 2;
    } else {
        medianRight = rightLane[sizeRight / 2].x();
    }

    this->leftLaneOffset = medianLeft;
    this->rightLaneOffset = medianRight;
}

const double &LaneSteeringCalculator::getLeftLaneOffset(){
    return leftLaneOffset;
}

const double &LaneSteeringCalculator::getRightLaneOffset(){
    return rightLaneOffset;
}

const HTWKPoint &LaneSteeringCalculator::getPictureCenter(){
    return pictureCenter;
}

void LaneSteeringCalculator::resetLanePoints() {
    this->leftLane.clear();
    this->rightLane.clear();
}
