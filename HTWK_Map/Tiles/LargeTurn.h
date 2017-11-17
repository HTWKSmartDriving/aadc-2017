//
// Created by fabian on 11.08.17.
//

#ifndef AADC_USER_LARGETURN_H
#define AADC_USER_LARGETURN_H

#define LARGE_TURN_TILE_SIZE 3

#include "Turn.h"

class LargeTurn : public Turn {
public:
    LargeTurn(int id, MapTileType tileTypeId, const HTWKPoint &positon, Orientation::MapTile orient) : Turn(id,
                                                                                                            tileTypeId,
                                                                                                            positon,
                                                                                                            orient,
                                                                                                            LARGE_TURN_TILE_SIZE) {}
};


#endif //AADC_USER_LARGETURN_H