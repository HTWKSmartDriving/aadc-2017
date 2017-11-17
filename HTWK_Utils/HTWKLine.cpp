//
// Created by nick on 09.11.17.
//

#include "HTWKLine.h"

HTWKLine::HTWKLine(tFloat slope, tFloat shift) {
    slopeVal = slope;
    shiftVal = shift;
    constXVal = 0;
}

HTWKLine::HTWKLine(HTWKPoint p1, HTWKPoint p2) {
    if (p1.x() == p2.x()) {
        slopeVal = 0;
        shiftVal = 0;
        constXVal = p1.x();
    } else {
        slopeVal = (p1.y() - p2.y()) / (p1.x() - p2.x());
        shiftVal = p1.y() - (slopeVal * p1.x());
    }
}

tFloat HTWKLine::slope() const {
    return slopeVal;
}

tFloat HTWKLine::shift() const {
    return shiftVal;
}

tFloat HTWKLine::constX() const {
    return constXVal;
}


void HTWKLine::setSlope(double slope) {
    slopeVal = slope;
    constXVal = 0;
}

void HTWKLine::setShift(double shift) {
    shiftVal = shift;
    constXVal = 0;
}

void HTWKLine::setConstX(double constX) {
    shiftVal = 0;
    slopeVal = 0;
    constXVal = constX;
}

double HTWKLine::dist(const HTWKPoint &point) const {

    if (constXVal != 0) {
        return abs(point.x() - constXVal);
    } else {
        HTWKLine perpendicular;      //Senkrechte
        perpendicular.setSlope(-1 / slopeVal);
        perpendicular.setShift(point.y() - perpendicular.slope());
        HTWKPoint intersection;
        intersection.setXval((perpendicular.shift() - shiftVal) / (slopeVal - perpendicular.slope()));
        intersection.setYval((slopeVal * intersection.x() + shiftVal));
        return std::sqrt(Square(intersection.x() - point.x()) + Square(intersection.y() - point.y()));
    }
}

tFloat HTWKLine::Square(tFloat value) const {
    return value * value;
}
