//
// Created by fabian on 08.08.17.
//

#ifndef AADC_USER_CARTRAJECTORY_H
#define AADC_USER_CARTRAJECTORY_H

#include <vector>
#include <iostream>
#include "math.h"
#include "../HTWK_Utils/HTWKPoint.hpp"
#include <cfloat>
#include "../HTWK_Debug/EnableLogs.h"

#define WHEEL_BASE 0.363f
#define COORDINATE_CORRECTION 90
#define PLAN_AHEAD_DISTANCE 0.9f

# define PI 3.14159265358979323846f

class CarTrajectory {
public:
    void getTrajectoryPoints(const float &angle, const HTWKPoint &position, const float &orientation,
                             const float &stepSize, std::vector<HTWKPoint> &points);

    double getTrajectoryScore(const std::vector<HTWKPoint> &pointVector1, const std::vector<HTWKPoint> &pointVector2);

private:
    void straight(std::vector<HTWKPoint> &points, const HTWKPoint &point, const float &orientation,
                      const float &stepSize, const float &distance);

    void arch(std::vector<HTWKPoint> &points, const float &angle, const HTWKPoint &point, const float &orientation,
              const float &stepSize);

    void rotate(std::vector<HTWKPoint> &vector, const float &orientation, const HTWKPoint &point);
};

#endif //AADC_USER_CARTRAJECTORY_H
