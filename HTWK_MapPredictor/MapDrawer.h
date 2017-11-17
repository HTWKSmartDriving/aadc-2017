
#ifndef AADC_USER_MAPDRAWER_H
#define AADC_USER_MAPDRAWER_H

#define EMPTY_TILE "."

#include <utility>
#include <vector>
#include <map>

#include "./DataTypes.h"
#include "../HTWK_Map/MapElement.h"

class MapDrawer {
public:
    MapDrawer(std::map<int, std::map<int, std::vector<MP::roadSign>>> signsMap,
              std::map<int, std::map<int, MapElement>> tilesMap,
              const MP::bounds &bounds) : signsMap(std::move(signsMap)), tilesMap(std::move(tilesMap)), bounds(bounds) {};

    void drawMap(bool drawSigns);

private:
    std::map<int, std::map<int, std::vector<MP::roadSign>>> signsMap;
    std::map<int, std::map<int, MapElement>> tilesMap;
    MP::bounds bounds;
};


#endif //AADC_USER_MAPDRAWER_H
