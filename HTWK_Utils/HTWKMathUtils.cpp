#include "HTWKMathUtils.hpp"

double HTWKMathUtils::gainByTwoPoints(const HTWKPoint &a, const HTWKPoint &b)
{
  double diffy = b.y() - a.y();
  double diffx = b.x() - a.x();

  if ((diffx < 0.0001 && diffx > -0.0001) && diffy < 0)
  {
    return -FLT_MAX;
  } else if ((diffx < 0.0001 && diffx > -0.0001) && diffy > 0)
  {
    return FLT_MAX;
  } else
  {
    return (diffy) / (diffx);
  }
}

void HTWKMathUtils::circleIntersections(double radius1,
                                        HTWKPoint center1,
                                        double radius2,
                                        HTWKPoint center2,
                                        HTWKPoint &intersection1,
                                        HTWKPoint &intersection2)
{
  double c = center2.dist(center1);

  if (c == 0)
  {
    return;
  }

  double xoff = (radius1 * radius1 + c * c - radius2 * radius2) / (2 * c);
  double yoff = radius1 * radius1 - xoff * xoff;

  if (yoff < 0)
  {
    return;
  }

  yoff = sqrt(yoff);

  intersection1 =
      HTWKPoint(center1.x() + xoff * ((center2.x() - center1.x()) / c) - yoff * ((center2.y() - center1.y()) / c),
                center1.y() + xoff * ((center2.y() - center1.y()) / c) + yoff * ((center2.x() - center1.x()) / c));

  if (yoff == 0)
  {
    return;
  }

  intersection2 =
      HTWKPoint(center1.x() + xoff * ((center2.x() - center1.x()) / c) + yoff * ((center2.y() - center1.y()) / c),
                center1.y() + xoff * ((center2.y() - center1.y()) / c) - yoff * ((center2.x() - center1.x()) / c));
}

double HTWKMathUtils::angleBy3Points(HTWKPoint pointMid, HTWKPoint point1, HTWKPoint point2, bool oppositeAngle)
{
    double vec1[2];
    double vec2[2];

    vec1[0] = point1.x() - pointMid.x();
    vec2[0] = point2.x() - pointMid.x();

    vec1[1] = point1.y() - pointMid.y();
    vec2[1] = point2.y() - pointMid.y();

    if((point2.y() < pointMid.y()) && oppositeAngle) {
        return 360 - (acos((vec1[0]*vec2[0]+vec1[1]*vec2[1])/(point1.dist(pointMid)*point2.dist(pointMid))) * 180/M_PI);
    } else {
        return (acos((vec1[0]*vec2[0]+vec1[1]*vec2[1])/(point1.dist(pointMid)*point2.dist(pointMid))) * 180/M_PI);
    }
}

double HTWKMathUtils::quatToYaw(double qw, double qx, double qy, double qz)
{
  if (qz > 0 && qz <= qw)
  {
    return asin(2 * qx * qy + 2 * qz * qw) * 180 / M_PI;
  } else if (qz > 0 && qz > qw)
  {
    return (180 - (asin(2 * qx * qy + 2 * qz * qw) * 180 / M_PI));
  } else if (qz < 0 && fabs(qz) > qw)
  {
    return (-180 - (asin(2 * qx * qy + 2 * qz * qw) * 180 / M_PI));
  } else if (qz < 0 && fabs(qz) < qw)
  {
    return asin(2 * qx * qy + 2 * qz * qw) * 180 / M_PI;
  } else
  {
    return 0;
  }
}

double HTWKMathUtils::rad2deg(const double &rad) {
    return (rad * 180 / PI);
}

const double HTWKMathUtils::get3DVectorNorm(const double &e1, const double &e2, const double &e3) {
    return sqrt(e1 * e1 + e2 * e2 + e3 * e3);
}

