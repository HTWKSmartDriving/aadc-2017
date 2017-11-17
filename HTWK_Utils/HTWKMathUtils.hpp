#ifndef HTWKUTILS_HTWKMATHUTILS_H
#define HTWKUTILS_HTWKMATHUTILS_H

#include "HTWKPoint.hpp"
#include <cstdio>
#include <vector>
#include <cfloat>

#define PI 3.14159265358979323846f

class HTWKMathUtils
{
 public:
    static double gainByTwoPoints(const HTWKPoint &a, const HTWKPoint &b);

  static void circleIntersections(double radius1,
                                  HTWKPoint center1,
                                  double radius2,
                                  HTWKPoint center2,
                                  HTWKPoint &intersection1,
                                  HTWKPoint &intersection2);

  static double angleBy3Points(HTWKPoint pointMid, HTWKPoint point1, HTWKPoint point2, bool oppositeAngle = true);

  static double quatToYaw(double qw, double qx, double qy, double qz);

    static double rad2deg(const double &rad);

/*  static int median(std::vector<int> v)
  {
    size_t n = v.size() / 2;
    nth_element(v.begin(), v.begin() + n, v.end());
    return v[n];
  }
*/
    static const double get3DVectorNorm(const double &e1, const double &e2, const double &e3);


};

#endif //HTWKUTILS_HTWKMATHUTILS_H
