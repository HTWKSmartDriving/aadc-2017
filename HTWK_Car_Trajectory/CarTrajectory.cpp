//
// Created by fabian on 08.08.17.
//

#include "CarTrajectory.h"

void
CarTrajectory::getTrajectoryPoints(const float &angle, const HTWKPoint &position, const float &orientation,
                                   const float &stepSize, std::vector<HTWKPoint> &points) {

    HTWKPoint positionWithCarsize = position.moveNewPointByDistanceAndAngleDeg(WHEEL_BASE/2.0f, orientation);

    if (angle == 0) {
        straight(points, positionWithCarsize, 0, stepSize, PLAN_AHEAD_DISTANCE);
        rotate(points, orientation, positionWithCarsize);
    } else {
        arch(points, angle, positionWithCarsize, 0, stepSize);
        rotate(points, orientation - 90, positionWithCarsize);
    }
}

void CarTrajectory::straight(std::vector<HTWKPoint> &points, const HTWKPoint &position, const float &orientation,
                             const float &stepSize, const float &distance) {

    float sampleValue = 0;
    while (sampleValue < distance) {
        points.emplace_back(
                position.moveNewPointByDistanceAndAngleDeg(sampleValue, orientation));
        sampleValue += stepSize;
    }

}

void CarTrajectory::arch(std::vector<HTWKPoint> &points, const float &angle, const HTWKPoint &position,
                         const float &orientation, const float &stepSize) {
    auto radius = static_cast<float>(WHEEL_BASE / sin(angle * PI / 180));
    HTWKPoint curveCenter;
    curveCenter = position.moveNewPointByDistanceAndAngleDeg(radius, 180);

/*    std::cout << "angle: " << angle << std::endl;
    std::cout << "curve-radius: " << radius << std::endl;
    std::cout << "curve-center: x:" << curveCenter.x() << "y: " << curveCenter.y() << std::endl;*/

    float sampleValue = 0;
    while (sampleValue < PLAN_AHEAD_DISTANCE) {
        float centerAngle = sampleValue / radius;

        if (std::abs(centerAngle) > PI / 2)
            break;

        HTWKPoint newPoint = curveCenter.moveNewPointByDistanceAndAngleRad(radius, centerAngle);
        points.emplace_back(newPoint);
        //std::cout << "Points: x:" << newPoint.x() << "y: " << newPoint.y() << std::endl;

        sampleValue += stepSize;
    }
}

void CarTrajectory::rotate(std::vector<HTWKPoint> &vector, const float &orientation, const HTWKPoint &position) {
    for (auto &value: vector) {
        double rotatedX = position.x() + cos(orientation * PI / 180) * (value.x() - position.x()) -
                          sin(orientation * PI / 180) * (value.y() - position.y());
        double rotatedY = position.y() + sin(orientation * PI / 180) * (value.x() - position.x()) +
                          cos(orientation * PI / 180) * (value.y() - position.y());

        value.setXval(rotatedX);
        value.setYval(rotatedY);
    }
}

double CarTrajectory::getTrajectoryScore(const std::vector<HTWKPoint> &pointVector1,
                                         const std::vector<HTWKPoint> &pointVector2) {

    double score = 0;

    for (const auto &pointInVector1: pointVector1) {
        auto shortest = DBL_MAX;

        for (const auto &pointInVector2: pointVector2) {
            double distance = pointInVector1.dist(pointInVector2);
            if (distance < shortest) {
                shortest = distance;
            }
        }

        score += shortest;
    }

    return score;
}
