#ifndef PROJECT_LANE_STEERING_H
#define PROJECT_LANE_STEERING_H

#include <stdint.h>
#include <utility>
#include <a_utils.h>
#include <algorithm>
#include "../../HTWK_Utils/HTWKPoint.hpp"
#include "../../HTWK_Utils/HTWKMathUtils.hpp"
#include "../../HTWK_Types/LaneData.h"
#include "../../HTWK_Debug/EnableLogs.h"
#define HALF 2

class LaneSteeringCalculator {
public:
    LaneSteeringCalculator(const int frameWidth, const int frameHeight);

    void calcSteering(const std::vector<tLanePoint> &tLanePointVector);

    const double &getLeftLaneOffset();

    const double &getRightLaneOffset();

    const HTWKPoint &getPictureCenter();

private:

    void iteratePoints(const std::vector<tLanePoint> &vector);

    void updateLanePoints(const tLanePoint &point);

    void resetLanePoints();

    void calcSteeringAngle();

    std::vector<HTWKPoint> leftLane;
    std::vector<HTWKPoint> rightLane;
    HTWKPoint pictureCenter;

    double leftLaneOffset;
    double rightLaneOffset;
};

#endif //PROJECT_LANE_STEERING_H
