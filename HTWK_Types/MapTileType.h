#ifndef AADC_USER_MAPTILE_H
#define AADC_USER_MAPTILE_H

#include <map>

enum class MapTileType {
    NOT_SET = -1, // used in MapPredictor (implies NULL value)
    STRAIGHT = 1,
    INTERSECTION_T = 2,
    INTERSECTION_PLUS = 3,
    TURN_LARGE = 4,
    TURN_SMALL = 5,
    LOT = 7,
    TURN_S_LEFT = 8,
    TURN_S_RIGHT = 9
};

static const std::map<const MapTileType, const std::string> MAP_TILE_TYPE_NAME_MAP = {
    {MapTileType::NOT_SET, "Missing"},
    {MapTileType::STRAIGHT, "Straight"},
    {MapTileType::INTERSECTION_T, "Threeway crossing"},
    {MapTileType::INTERSECTION_PLUS, "Plus crossing"},
    {MapTileType::TURN_LARGE, "Large turn"},
    {MapTileType::TURN_SMALL, "Small turn"},
    {MapTileType::LOT, "Lot"},
    {MapTileType::TURN_S_LEFT, "S turn left"},
};

inline const std::string mapTileTypeToString(const MapTileType &id) {
    auto it = MAP_TILE_TYPE_NAME_MAP.find(id);
    if (it != MAP_TILE_TYPE_NAME_MAP.end()) {
        return it->second;
    } else {
        return std::string("No Valid ID");
    }
};

#endif //AADC_USER_MAPTILE_H
