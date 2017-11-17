//
// Created by fabian on 06.08.17.
//

#ifndef AADC_USER_ORIENTATION_H
#define AADC_USER_ORIENTATION_H


namespace Orientation {
    enum class MapTile {
        NORTH = 90,
        WEST = 180,
        EAST = 0,
        SOUTH = -90
    };

    enum class Global {
        NORTH = 1,
        WEST = 2,
        EAST = 0,
        SOUTH = 3
    };

    enum class TurnDirection {
        STRAIGHT = 0,
        LEFT = 1,
        RIGHT = 2
    };

    bool inline isVertical(MapTile tile) {
        return tile == MapTile::NORTH || tile == MapTile::SOUTH;
    }

    bool inline isHorizontal(MapTile tile) {
        return tile == MapTile::EAST || tile == MapTile::WEST;
    }
}


#endif //AADC_USER_ORIENTATION_H
