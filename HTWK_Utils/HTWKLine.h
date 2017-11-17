//
// Created by nick on 09.11.17.
//

#ifndef HTWKUTILS_HTWKLINE_H
#define HTWKUTILS_HTWKLINE_H


#include <a_utils.h>
#include "HTWKPoint.hpp"

class HTWKLine {

private:
    tFloat slopeVal = 0;
    tFloat shiftVal = 0;
    tFloat constXVal = 0;

    tFloat Square(tFloat value) const;

public:
    explicit HTWKLine(tFloat slope = 0.0, tFloat shift = 0);

    HTWKLine(HTWKPoint p1, HTWKPoint p2);

    tFloat slope() const;

    tFloat shift() const;

    tFloat constX() const;

    double dist(const HTWKPoint &point) const;

    void setSlope(double slope);

    void setShift(double shift);

    void setConstX(double constX);

};


#endif //AADC_USER_HTWKLINE_H
