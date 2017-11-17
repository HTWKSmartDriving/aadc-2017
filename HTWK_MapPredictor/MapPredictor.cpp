#include "MapPredictor.h"

std::map<MP::DETECTION_TYPE, float> MP::PROPABILITY_MAP = {
    {DETECTION_TYPE::PLUS_CROSSING,             1},
    {DETECTION_TYPE::THREEWAY_CROSSING,         1},
    {DETECTION_TYPE::STRAIGHT_NEXT_TO_CROSSING, 0.9},
    {DETECTION_TYPE::STRAIGHT_NEXT_TO_PARKING,  1},
    {DETECTION_TYPE::STRAIGHT_GAP,              0.5},
    {DETECTION_TYPE::ZEBRA,                     1},
    {DETECTION_TYPE::TURN_SMALL,                0.25},
    {DETECTION_TYPE::TURN_LARGE,                0.25},
};

/**
 *  Ideen
 *  - Fahrmanöver miteinbeziehen?
 *  - Generell stehen Schilder oft neben geraden Streckenstücken
 */

MapPredictor::MapPredictor(const char *signsXmlFile) {
    loadXmlData(signsXmlFile);
}

std::map<int, std::map<int, MapElement>> MapPredictor::getPredictedMap() {
    if (!tilesPredicted)
        predictTiles();

    return tilesMap;
}

// ------

int MapPredictor::getNextId() {
    return curId++;
}

float MapPredictor::getProbability(const MP::DETECTION_TYPE &id) {
    auto it = MP::PROPABILITY_MAP.find(id);
    if (it != MP::PROPABILITY_MAP.end()) {
        return it->second;
    } else {
        return -1;
    }
}

// TODO: this function does too many things
void MapPredictor::loadXmlData(const char *fileName) {
    tinyxml2::XMLDocument xmlDoc;
    tinyxml2::XMLError result = xmlDoc.LoadFile(fileName);

    if (result != tinyxml2::XML_SUCCESS) {
        std::cerr << "Unable to load XML file" << std::endl;
        exit(EXIT_FAILURE);
    }

    int type, intX, intY, dir;
    double x, y;

    // TODO: the tiles need to be also considered!
    // TODO: add the map object size

    // road signs
    tinyxml2::XMLElement *startElem = xmlDoc.RootElement()->FirstChildElement(TAG_NAME_SIGN);
    for (tinyxml2::XMLElement *elem = startElem; elem != nullptr; elem = elem->NextSiblingElement(TAG_NAME_SIGN)) {
        elem->QueryAttribute("x", &x);
        elem->QueryAttribute("y", &y);
        elem->QueryAttribute("id", &type);

        intX = static_cast<int>(floor(x));
        intY = static_cast<int>(floor(y));

        MP::roadSign sign = {static_cast<ROAD_SIGN>(type), HTWKPoint(x, y), intX, intY};
        signsMap[intX][intY].push_back(sign);

        updateBounds(intX, intY);
    }

    // parking lots
    startElem = xmlDoc.RootElement()->FirstChildElement(TAG_NAME_PARKING);
    for (tinyxml2::XMLElement *elem = startElem; elem != nullptr; elem = elem->NextSiblingElement(TAG_NAME_PARKING)) {
        elem->QueryAttribute("x", &x);
        elem->QueryAttribute("y", &y);
        elem->QueryAttribute("direction", &dir);

        // parking lots seem to overlap tiles often
        auto minX = static_cast<int>(floor(x));
        auto minY = static_cast<int>(floor(y));
        auto maxX = static_cast<int>(ceil(x));
        auto maxY = static_cast<int>(ceil(y));

        // parkin lots imply straight streets nearby
        const float prob  = getProbability(MP::DETECTION_TYPE::STRAIGHT_NEXT_TO_PARKING);
        auto fixedOrient = static_cast<Orientation::MapTile>(dir);
        fixedOrient = Orientation::isVertical(fixedOrient) ? Orientation::MapTile::EAST : Orientation::MapTile::NORTH;
        setTile(MP::pos{minX, minY}, MapTileType::STRAIGHT, fixedOrient, prob, true);
        setTile(MP::pos{maxX, maxY}, MapTileType::STRAIGHT, fixedOrient, prob, true);
        updateBounds(minX, minY);
        updateBounds(maxX, maxY);
    }

    // pedestrian crossings
    startElem = xmlDoc.RootElement()->FirstChildElement(TAG_NAME_ZEBRA);
    for (tinyxml2::XMLElement *elem = startElem; elem != nullptr; elem = elem->NextSiblingElement(TAG_NAME_ZEBRA)) {
        elem->QueryAttribute("x", &x);
        elem->QueryAttribute("y", &y);
        elem->QueryAttribute("direction", &dir);

        intX = static_cast<int>(floor(x));
        intY = static_cast<int>(floor(y));

        // a pedestrian crossing (aka zebra) implies a straight road
        const float prob = getProbability(MP::DETECTION_TYPE::ZEBRA);
        setTile(MP::pos{intX, intY}, MapTileType::STRAIGHT, static_cast<Orientation::MapTile>(dir), prob, true);
        updateBounds(intX, intY);
    }
}

void MapPredictor::updateBounds(const int x, const int y) {
    bounds.maxX = std::max(bounds.maxX, x);
    bounds.maxY = std::max(bounds.maxY, y);
    bounds.minX = std::min(bounds.minX, x);
    bounds.minY = std::min(bounds.minY, y);
}

void MapPredictor::setTile(const MP::pos &pos, const MapTileType tileType, const Orientation::MapTile &orientation,
                           const float probability, const bool overwrite) {
    auto curType = mapTileTypeToString(tilesMap[pos.x][pos.y].getTileTypeId());
    auto newType = mapTileTypeToString(tileType);

    if (tilesMap[pos.x][pos.y].isSet()) {
        if (overwrite) {
#ifdef DEBUG_MAP_PREDICTOR
                fprintf(stderr, "WARNING: Overwriting tile at %d/%d: %s <- %s\n", pos.x, pos.y, curType.c_str(), newType.c_str());
#endif
            tilesMap[pos.x][pos.y] = MapElement(getNextId(), tileType, HTWKPoint(pos.x, pos.y), orientation, probability);
        } else {
#ifdef DEBUG_MAP_PREDICTOR
            fprintf(stderr, "NOTE: I won't overwrite %d/%d: %s <- %s\n", pos.x, pos.y, curType.c_str(), newType.c_str());
#endif
        }
    } else {
#ifdef DEBUG_MAP_PREDICTOR
            fprintf(stdout, "Setting tile at %d/%d: %s <- %s\n", pos.x, pos.y, curType.c_str(), newType.c_str());
#endif
        tilesMap[pos.x][pos.y] = MapElement(getNextId(), tileType, HTWKPoint(pos.x, pos.y), orientation, probability);
    }
}

bool MapPredictor::mapContains(const MP::pos &pos, const ROAD_SIGN signType) {
    auto signs = signsMap[pos.x][pos.y];
    auto pred = [signType](const MP::roadSign &sign) {
        return sign.type == signType;
    };
    return std::find_if(std::begin(signs), std::end(signs), pred) != std::end(signs);
}

bool MapPredictor::tilesAreEmpty(MP::pos pos, int dist) {
    return tilesAreEmpty(pos, dist, dist);
}

bool MapPredictor::tilesAreEmpty(MP::pos pos, const int xDist, const int yDist) {
    bool isEmpty = true;
    for (int x = pos.x; x < (pos.x + xDist); x++) {
        for (int y = pos.y; y < (pos.y + yDist); y++) {
            isEmpty &= !tilesMap[x][y].isSet();
        }
    }

    return isEmpty;
}

void MapPredictor::predictTiles() {
    for (const auto &x: signsMap) {
        for (const auto &y : x.second) {
            for (const auto &sign: y.second) {
                detectPlusCrossings(sign);
                detectThreewayCrossings(sign);
            }
        }
    }

    detectStraightsByTile();
    detectStraightsByStraights();
    detectSmallTurns();

    tilesPredicted = true;
}

bool MapPredictor::detectPlusCrossings(const MP::roadSign &sign) {
    /**
     *   Check all four possible quadrants around [B]
     *   for the typical sign pattern:
     *
     *   S  ~ ~ HW  ~ ~  S
     *   ~       ~       ~
     *   ~       ~       ~
     *   HW ~ ~ [S] ~ ~ HW
     *   ~       ~       ~
     *   ~       ~       ~
     *   S  ~ ~ HW  ~ ~  S
    */

    bool Q1 = false;
    bool Q2 = false;
    bool Q3 = false;
    bool Q4 = false;

    // First case (see figure above)
    if (sign.type == ROAD_SIGN::STOP) {
        Q1 = mapContains(MP::pos{sign.x, sign.y + SIGN_OFFSET}, ROAD_SIGN::HAVE_WAY) &&
             mapContains(MP::pos{sign.x + SIGN_OFFSET, sign.y}, ROAD_SIGN::HAVE_WAY) &&
             mapContains(MP::pos{sign.x + SIGN_OFFSET, sign.y + SIGN_OFFSET}, ROAD_SIGN::STOP);

        Q2 = mapContains(MP::pos{sign.x, sign.y + SIGN_OFFSET}, ROAD_SIGN::HAVE_WAY) &&
             mapContains(MP::pos{sign.x - SIGN_OFFSET, sign.y}, ROAD_SIGN::HAVE_WAY) &&
             mapContains(MP::pos{sign.x - SIGN_OFFSET, sign.y + SIGN_OFFSET}, ROAD_SIGN::STOP);

        Q3 = mapContains(MP::pos{sign.x, sign.y - SIGN_OFFSET}, ROAD_SIGN::HAVE_WAY) &&
             mapContains(MP::pos{sign.x - SIGN_OFFSET, sign.y}, ROAD_SIGN::HAVE_WAY) &&
             mapContains(MP::pos{sign.x - SIGN_OFFSET, sign.y - SIGN_OFFSET}, ROAD_SIGN::STOP);

        Q4 = mapContains(MP::pos{sign.x, sign.y - SIGN_OFFSET}, ROAD_SIGN::HAVE_WAY) &&
             mapContains(MP::pos{sign.x + SIGN_OFFSET, sign.y}, ROAD_SIGN::HAVE_WAY) &&
             mapContains(MP::pos{sign.x + SIGN_OFFSET, sign.y - SIGN_OFFSET}, ROAD_SIGN::STOP);
    }

        // Second case (S and HW exchanged)
    else if (sign.type == ROAD_SIGN::HAVE_WAY) {
        Q1 = mapContains(MP::pos{sign.x, sign.y + SIGN_OFFSET}, ROAD_SIGN::STOP) &&
             mapContains(MP::pos{sign.x + SIGN_OFFSET, sign.y}, ROAD_SIGN::STOP) &&
             mapContains(MP::pos{sign.x + SIGN_OFFSET, sign.y + SIGN_OFFSET}, ROAD_SIGN::HAVE_WAY);

        Q2 = mapContains(MP::pos{sign.x, sign.y + SIGN_OFFSET}, ROAD_SIGN::STOP) &&
             mapContains(MP::pos{sign.x - SIGN_OFFSET, sign.y}, ROAD_SIGN::STOP) &&
             mapContains(MP::pos{sign.x - SIGN_OFFSET, sign.y + SIGN_OFFSET}, ROAD_SIGN::HAVE_WAY);

        Q3 = mapContains(MP::pos{sign.x, sign.y - SIGN_OFFSET}, ROAD_SIGN::STOP) &&
             mapContains(MP::pos{sign.x - SIGN_OFFSET, sign.y}, ROAD_SIGN::STOP) &&
             mapContains(MP::pos{sign.x - SIGN_OFFSET, sign.y - SIGN_OFFSET}, ROAD_SIGN::HAVE_WAY);

        Q4 = mapContains(MP::pos{sign.x, sign.y - SIGN_OFFSET}, ROAD_SIGN::STOP) &&
             mapContains(MP::pos{sign.x + SIGN_OFFSET, sign.y}, ROAD_SIGN::STOP) &&
             mapContains(MP::pos{sign.x + SIGN_OFFSET, sign.y - SIGN_OFFSET}, ROAD_SIGN::HAVE_WAY);
    }

    const float prob = getProbability(MP::DETECTION_TYPE::PLUS_CROSSING);
    if (Q1) setTile(MP::pos{sign.x + 1, sign.y + 1}, MapTileType::INTERSECTION_PLUS, Orientation::MapTile::NORTH, prob, true);
    if (Q2) setTile(MP::pos{sign.x - 1, sign.y + 1}, MapTileType::INTERSECTION_PLUS, Orientation::MapTile::NORTH, prob, true);
    if (Q3) setTile(MP::pos{sign.x - 1, sign.y - 1}, MapTileType::INTERSECTION_PLUS, Orientation::MapTile::NORTH, prob, true);
    if (Q4) setTile(MP::pos{sign.x + 1, sign.y - 1}, MapTileType::INTERSECTION_PLUS, Orientation::MapTile::NORTH, prob, true);

    return Q1 || Q2 || Q3 || Q4;
}

bool MapPredictor::detectThreewayCrossings(const MP::roadSign &sign) {
    // Ensure that at base position is a STOP or GIVE_WAY sign (= S/GW)
    if (sign.type != ROAD_SIGN::STOP && sign.type != ROAD_SIGN::GIVE_WAY) {
        return false;
    }

    /**
     *   S/GW ~ ~ HW
     *   ~
     *   ~
     *   HW      ( )
     */
    bool caseN = mapContains(MP::pos{sign.x + SIGN_OFFSET, sign.y}, ROAD_SIGN::HAVE_WAY) &&
                 mapContains(MP::pos{sign.x, sign.y - SIGN_OFFSET}, ROAD_SIGN::HAVE_WAY) &&
                 signsMap[sign.x + SIGN_OFFSET][sign.y - SIGN_OFFSET].empty();

    /**
     *   HW ~ ~ S/GW
     *             ~
     *             ~
     *   ( )      HW
     */
    bool caseE = mapContains(MP::pos{sign.x - SIGN_OFFSET, sign.y}, ROAD_SIGN::HAVE_WAY) &&
                 mapContains(MP::pos{sign.x, sign.y - SIGN_OFFSET}, ROAD_SIGN::HAVE_WAY) &&
                 signsMap[sign.x - SIGN_OFFSET][sign.y - SIGN_OFFSET].empty();

    /**
     *   ( )      HW
     *             ~
     *             ~
     *   HW ~ ~ S/GW
     */
    bool caseS = mapContains(MP::pos{sign.x - SIGN_OFFSET, sign.y}, ROAD_SIGN::HAVE_WAY) &&
                 mapContains(MP::pos{sign.x, sign.y + SIGN_OFFSET}, ROAD_SIGN::HAVE_WAY) &&
                 signsMap[sign.x - SIGN_OFFSET][sign.y + SIGN_OFFSET].empty();

    /**
    *   HW      ( )
    *   ~
    *   ~
    *   S/GW ~ ~ HW
    *
    */
    bool caseW = mapContains(MP::pos{sign.x + SIGN_OFFSET, sign.y}, ROAD_SIGN::HAVE_WAY) &&
                 mapContains(MP::pos{sign.x, sign.y + SIGN_OFFSET}, ROAD_SIGN::HAVE_WAY) &&
                 signsMap[sign.x + SIGN_OFFSET][sign.y + SIGN_OFFSET].empty();

    const float prob = getProbability(MP::DETECTION_TYPE::THREEWAY_CROSSING);
    if (caseN) setTile(MP::pos{sign.x + 1, sign.y - 1}, MapTileType::INTERSECTION_T, Orientation::MapTile::NORTH, prob, true);
    if (caseE) setTile(MP::pos{sign.x - 1, sign.y - 1}, MapTileType::INTERSECTION_T, Orientation::MapTile::EAST,  prob, true);
    if (caseS) setTile(MP::pos{sign.x - 1, sign.y + 1}, MapTileType::INTERSECTION_T, Orientation::MapTile::SOUTH, prob, true);
    if (caseW) setTile(MP::pos{sign.x + 1, sign.y + 1}, MapTileType::INTERSECTION_T, Orientation::MapTile::WEST,  prob, true);

    return caseN || caseE || caseS || caseW;
}

void MapPredictor::detectStraightsByTile() {
    for (const auto &x: tilesMap) {
        for (const auto &y : x.second) {
            MP::pos pos = {x.first, y.first};

            auto tile = tilesMap[pos.x][pos.y];
            auto type = tile.getTileTypeId();

            if (type == MapTileType::INTERSECTION_PLUS) {
                const float prob = getProbability(MP::DETECTION_TYPE::STRAIGHT_NEXT_TO_CROSSING);
                setTile({pos.x, pos.y + 1}, MapTileType::STRAIGHT, Orientation::MapTile::NORTH, prob);
                setTile({pos.x + 1, pos.y}, MapTileType::STRAIGHT, Orientation::MapTile::EAST,  prob);
                setTile({pos.x, pos.y - 1}, MapTileType::STRAIGHT, Orientation::MapTile::SOUTH, prob);
                setTile({pos.x - 1, pos.y}, MapTileType::STRAIGHT, Orientation::MapTile::WEST,  prob);
            }
            else if (type == MapTileType::INTERSECTION_T) {
                const float prob = getProbability(MP::DETECTION_TYPE::STRAIGHT_NEXT_TO_CROSSING);
                switch (tile.getOrient()) {
                    case Orientation::MapTile::NORTH:
                        setTile({pos.x, pos.y + 1}, MapTileType::STRAIGHT, Orientation::MapTile::NORTH, prob);
                        setTile({pos.x + 1, pos.y}, MapTileType::STRAIGHT, Orientation::MapTile::EAST,  prob);
                        setTile({pos.x - 1, pos.y}, MapTileType::STRAIGHT, Orientation::MapTile::WEST,  prob);
                        break;
                    case Orientation::MapTile::EAST:
                        setTile({pos.x, pos.y + 1}, MapTileType::STRAIGHT, Orientation::MapTile::NORTH, prob);
                        setTile({pos.x + 1, pos.y}, MapTileType::STRAIGHT, Orientation::MapTile::EAST,  prob);
                        setTile({pos.x, pos.y - 1}, MapTileType::STRAIGHT, Orientation::MapTile::SOUTH, prob);
                        break;
                    case Orientation::MapTile::SOUTH:
                        setTile({pos.x + 1, pos.y}, MapTileType::STRAIGHT, Orientation::MapTile::EAST,  prob);
                        setTile({pos.x, pos.y - 1}, MapTileType::STRAIGHT, Orientation::MapTile::SOUTH, prob);
                        setTile({pos.x - 1, pos.y}, MapTileType::STRAIGHT, Orientation::MapTile::WEST,  prob);
                        break;
                    case Orientation::MapTile::WEST:
                        setTile({pos.x, pos.y + 1}, MapTileType::STRAIGHT, Orientation::MapTile::NORTH, prob);
                        setTile({pos.x, pos.y - 1}, MapTileType::STRAIGHT, Orientation::MapTile::SOUTH, prob);
                        setTile({pos.x - 1, pos.y}, MapTileType::STRAIGHT, Orientation::MapTile::WEST,  prob);
                        break;
                }
            }
        }
    }
}

void MapPredictor::detectStraightsByStraights() {
    const float baseProb = getProbability(MP::DETECTION_TYPE::STRAIGHT_GAP);

    for (const auto &x: tilesMap) {
        for (const auto &y : x.second) {
            MP::pos pos = {x.first, y.first};

            auto tile = tilesMap[pos.x][pos.y];
            auto type = tile.getTileTypeId();

            if (type != MapTileType::STRAIGHT)
                continue;

            // ═  .  ═
            if (Orientation::isVertical(tile.getOrient())) {
                for (const int &offset : {-2, 2}) {
                    auto neighbour = tilesMap[pos.x][pos.y + offset];
                    if (neighbour.getTileTypeId() == MapTileType::STRAIGHT && Orientation::isVertical(neighbour.getOrient())) {
                        int offsetY = offset < 0 ? -1 : 1;
                        if (!tilesMap[pos.x][pos.y + offsetY].isSet()) {
                            setTile({pos.x, pos.y + offsetY}, MapTileType::STRAIGHT, Orientation::MapTile::NORTH,
                                    baseProb * neighbour.getProbability());
                        }
                    }
                }
            }

            // ║
            // .
            // ║
            else if (Orientation::isHorizontal(tile.getOrient())) {
                for (const int &offset : {-2, 2}) {
                    auto neighbour = tilesMap[pos.x + offset][pos.y];
                    if (neighbour.getTileTypeId() == MapTileType::STRAIGHT && Orientation::isHorizontal(neighbour.getOrient())) {
                        int offsetX = offset < 0 ? -1 : 1;
                        if (!tilesMap[pos.x + offsetX][pos.y].isSet()) {
                            setTile({pos.x + offsetX, pos.y}, MapTileType::STRAIGHT, Orientation::MapTile::EAST,
                                    baseProb * neighbour.getProbability());
                        }
                    }
                }
            }
        }
    }
}

void MapPredictor::detectSmallTurns() {
    const float baseProb = getProbability(MP::DETECTION_TYPE::TURN_SMALL);

    for (const auto &x: tilesMap) {
        for (const auto &y : x.second) {
            MP::pos pos = {x.first, y.first};

            auto tile = tilesMap[pos.x][pos.y];
            auto type = tile.getTileTypeId();

            if (type != MapTileType::STRAIGHT)
                continue;

            bool baseVertical = Orientation::isVertical(tile.getOrient());

            // ═  .  .  .  ═
            // .  .  .  .  .
            // .  .  ║  .  .
            // .  .  .  .  .
            // ═  .  .  .  ═
            //
            // (case with vertical base)

            if (tilesMap[pos.x + 2][pos.y + 2].getTileTypeId() == MapTileType::STRAIGHT &&
                Orientation::isHorizontal(tilesMap[pos.x + 2][pos.y + 2].getOrient())) {
                auto orient = baseVertical ? Orientation::MapTile::EAST : Orientation::MapTile::WEST;
                MP::pos turnPos = {pos.x, pos.y + 1};

                if (tilesAreEmpty(turnPos, TILE_SIZE_TURN_SMALL))
                    setTile(turnPos, MapTileType::TURN_SMALL, orient, baseProb * tile.getProbability());
            }

            else if (tilesMap[pos.x + 2][pos.y - 2].getTileTypeId() == MapTileType::STRAIGHT &&
                Orientation::isHorizontal(tilesMap[pos.x + 2][pos.y - 2].getOrient())) {
                auto orient = baseVertical ? Orientation::MapTile::NORTH : Orientation::MapTile::SOUTH;
                MP::pos turnPos = {pos.x, pos.y - 2};

                if (tilesAreEmpty(turnPos, TILE_SIZE_TURN_SMALL))
                    setTile(turnPos, MapTileType::TURN_SMALL, orient, baseProb * tile.getProbability());
            }

            else if (tilesMap[pos.x - 2][pos.y - 2].getTileTypeId() == MapTileType::STRAIGHT &&
                Orientation::isHorizontal(tilesMap[pos.x - 2][pos.y - 2].getOrient())) {
                auto orient = baseVertical ? Orientation::MapTile::WEST : Orientation::MapTile::EAST;
                MP::pos turnPos = {pos.x - 1, pos.y - 2};

                if (tilesAreEmpty(turnPos, TILE_SIZE_TURN_SMALL))
                    setTile(turnPos, MapTileType::TURN_SMALL, orient, baseProb * tile.getProbability());
            }

            else if (tilesMap[pos.x - 2][pos.y + 2].getTileTypeId() == MapTileType::STRAIGHT &&
                Orientation::isHorizontal(tilesMap[pos.x - 2][pos.y + 2].getOrient())) {
                auto orient = baseVertical ? Orientation::MapTile::SOUTH : Orientation::MapTile::NORTH;
                MP::pos turnPos = {pos.x - 1, pos.y + 1};

                if (tilesAreEmpty(turnPos, TILE_SIZE_TURN_SMALL))
                    setTile(turnPos, MapTileType::TURN_SMALL, orient, baseProb * tile.getProbability());
            }
        }
    }
}
