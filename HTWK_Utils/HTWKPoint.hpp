#ifndef HTWKUTILS_HTWKPOINT_H
#define HTWKUTILS_HTWKPOINT_H

#include <cmath>
#include <ostream>

#define HALF 2

// Class to represent points.
class HTWKPoint {
private:
    double xval = 0, yval = 0;

public:
    // Constructor uses default arguments to allow calling with zero, one,
    // or two values.
    explicit HTWKPoint(double x = 0.0, double y = 0.0);

    // Extractors.
    double x() const;

    double y() const;

    // Distance to another point.  Pythagorean thm.
    double dist(const HTWKPoint &other) const;

    // Add or subtract two points.
    HTWKPoint add(HTWKPoint b);

    HTWKPoint sub(HTWKPoint b);

    // Move existing point and create a new one
    HTWKPoint moveNewPointByXY(double a, double b) const;

    HTWKPoint moveNewPointByDistanceAndAngleRad(double dist, double angle) const;

    HTWKPoint moveNewPointByDistanceAndAngleDeg(double dist, double angle) const;

    static bool XGreater(HTWKPoint a, HTWKPoint b);

    HTWKPoint centerInX(HTWKPoint b);

    HTWKPoint centerInY(HTWKPoint b);

    void setXval(double xval);

    void setYval(double yval);

};

#endif //PROJECT_HTWKPOINT_H


