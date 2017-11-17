#include "HTWKPoint.hpp"


HTWKPoint::HTWKPoint(double x, double y)
{
  xval = x;
  yval = y;
}

double HTWKPoint::x() const
{
  return xval;
}

double HTWKPoint::y() const
{
  return yval;
}

double HTWKPoint::dist(const HTWKPoint &other) const
{
  double xd = xval - other.xval;
  double yd = yval - other.yval;
  return sqrt(xd * xd + yd * yd);
}

HTWKPoint HTWKPoint::sub(HTWKPoint b)
{
  return HTWKPoint(xval - b.xval, yval - b.yval);
}

HTWKPoint HTWKPoint::add(HTWKPoint b)
{
  return HTWKPoint(xval + b.xval, yval + b.yval);
}

HTWKPoint HTWKPoint::moveNewPointByXY(double a, double b) const
{
  return HTWKPoint(xval + a, yval + b);
}

HTWKPoint HTWKPoint::moveNewPointByDistanceAndAngleRad(double dist, double angle) const
{
  return HTWKPoint(xval + dist * cos(angle), yval + dist * sin(angle));
}

HTWKPoint HTWKPoint::moveNewPointByDistanceAndAngleDeg(double dist, double angle) const
{
  return moveNewPointByDistanceAndAngleRad(dist, angle * M_PI / 180);
}

HTWKPoint HTWKPoint::centerInX(HTWKPoint b)
{
  return HTWKPoint((xval + b.x())/HALF, yval);
}

HTWKPoint HTWKPoint::centerInY(HTWKPoint b)
{
    return HTWKPoint((xval + b.x())/HALF, yval);
}

void HTWKPoint::setXval(double xval) {
    HTWKPoint::xval = xval;
}

void HTWKPoint::setYval(double yval) {
    HTWKPoint::yval = yval;
}

bool HTWKPoint::XGreater(HTWKPoint a, HTWKPoint b) {
  return (a.x() > b.x());
}

