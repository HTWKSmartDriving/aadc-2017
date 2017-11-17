//
// Created by fabian on 13.08.17.
//

#ifndef AADC_USER_TILEPOINT_H
#define AADC_USER_TILEPOINT_H

#include "../HTWK_Utils/HTWKPoint.hpp"
#include "Orientation.h"
#include "MapElement.h"

class MapElement;

class TilePoint : public HTWKPoint {
public:
    explicit TilePoint(double x = 0.0, double y = 0.0, Orientation::Global edge = Orientation::Global::EAST)
            : HTWKPoint(x, y), edge(edge) {
    }

    explicit TilePoint(HTWKPoint point, Orientation::Global edge = Orientation::Global::EAST)
            : HTWKPoint(point), edge(edge) {}

public:
    Orientation::Global edge;

    //Every tilepoint can have up to 3 last and 3 next points (plus intersection)
    TilePoint *last[3]{nullptr, nullptr, nullptr};
    TilePoint *next[3]{nullptr, nullptr, nullptr};
};


#endif //AADC_USER_TILEPOINT_H
