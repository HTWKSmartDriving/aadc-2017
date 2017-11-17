#ifndef AADC_USER_DATATYPES_H
#define AADC_USER_DATATYPES_H

#include <map>

#include "../HTWK_Types/RoadSignEnum.h"
#include "../HTWK_Utils/HTWKPoint.hpp"

namespace MP {
    typedef struct {
        ROAD_SIGN type;
        HTWKPoint pos;
        int x;
        int y;
    } roadSign;

    typedef struct {
        int x;
        int y;
    } pos;

    typedef struct {
        int minX = 0;
        int minY = 0;
        int maxX = 0;
        int maxY = 0;
    } bounds;

    enum class DETECTION_TYPE {
        PLUS_CROSSING,
        THREEWAY_CROSSING,
        STRAIGHT_NEXT_TO_CROSSING,
        STRAIGHT_NEXT_TO_PARKING,
        STRAIGHT_GAP,
        ZEBRA,
        TURN_SMALL,
        TURN_LARGE,
    };

    extern std::map<DETECTION_TYPE, float> PROPABILITY_MAP;
}

#endif //AADC_USER_DATATYPES_H
