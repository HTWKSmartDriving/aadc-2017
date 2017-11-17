#ifndef AADC_USER_MAPPREDICTOR_H
#define AADC_USER_MAPPREDICTOR_H

#define DEBUG_MAP_PREDICTOR

#define TAG_NAME_SIGN "roadSign"
#define TAG_NAME_PARKING "parkingSpace"
#define TAG_NAME_ZEBRA "pedestrianCrossing"

#define SIGN_OFFSET 2

#define TILE_SIZE_TURN_SMALL 2
#define TILE_SIZE_TURN_LARGE 3
#define TILE_ID_START 1000

#include <iostream>
#include <algorithm>
#include <vector>
#include <map>
#include "tinyxml2.h"

#include "./DataTypes.h"
#include "../HTWK_Map/MapElement.h"

class MapPredictor {
public:
    explicit MapPredictor(const char *signsXmlFile);

    std::map<int, std::map<int, MapElement>> getPredictedMap();

    const std::map<int, std::map<int, std::vector<MP::roadSign>>> &getSignsMap() const {
        return signsMap;
    }

    const std::map<int, std::map<int, MapElement>> &getTilesMap() const {
        return tilesMap;
    }

    const MP::bounds &getBounds() const {
        return bounds;
    }

private:
    bool tilesPredicted = false;
    int curId = TILE_ID_START;

    std::map<int, std::map<int, std::vector<MP::roadSign>>> signsMap;
    std::map<int, std::map<int, MapElement>> tilesMap;

    MP::bounds bounds;

    int getNextId();

    float getProbability(const MP::DETECTION_TYPE &id);

    void loadXmlData(const char *fileName);

    void updateBounds(int x, int y);

    void setTile(const MP::pos &pos, MapTileType tileType, const Orientation::MapTile &orientation, float probability,
                 bool overwrite = false);

    bool mapContains(const MP::pos &pos, ROAD_SIGN signType);

    bool tilesAreEmpty(MP::pos pos, int dist);

    bool tilesAreEmpty(MP::pos pos, int xDist, int yDist);

    void predictTiles();

    bool detectPlusCrossings(const MP::roadSign &sign);

    bool detectThreewayCrossings(const MP::roadSign &sign);

    void detectStraightsByTile();

    void detectStraightsByStraights();

    void detectSmallTurns();
};


#endif //AADC_USER_MAPPREDICTOR_H
