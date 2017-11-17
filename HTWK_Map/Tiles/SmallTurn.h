//
// Created by fabian on 11.08.17.
//

#ifndef AADC_USER_SMALLTURN_H
#define AADC_USER_SMALLTURN_H

#include "Turn.h"

#define SMALL_TURN_TILE_SIZE 2

class SmallTurn : public Turn {
public:
    SmallTurn(int id, MapTileType tileTypeId, const HTWKPoint &positon, Orientation::MapTile orient) : Turn(id,
                                                                                                            tileTypeId,
                                                                                                            positon,
                                                                                                            orient,
                                                                                                            SMALL_TURN_TILE_SIZE) {}
};


#endif //AADC_USER_SMALLTURN_H
