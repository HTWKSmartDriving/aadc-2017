//
// Created by fabian on 11.08.17.
//

#ifndef AADC_USER_MAPELEMENT_H
#define AADC_USER_MAPELEMENT_H

#include <vector>
#include "../HTWK_Utils/HTWKPoint.hpp"
#include "../HTWK_Types/MapTileType.h"
#include "Orientation.h"
#include "TilePoint.h"

#define TRACK_WIDTH 0.44f
#define MIDDLE_LINE_WIDTH 0.02f
#define SIDE_LINE_WIDTH 0.03f
#define SPACE_NEXT_TO_SIDE_LINE_WIDTH 0.02f

#define INVALID_TILE -1

#pragma pack(push, 1)
class MapElement {
public:
    MapElement() = default;

    ~MapElement() {
        for(auto point : tileEntryPoints) {
            if(point != nullptr) {
                point = nullptr;
            }
        }

        for(auto point : tileExitPoints) {
            if(point != nullptr) {
                point = nullptr;
            }
        }
    }

    MapElement(int id, MapTileType tileTypeId, const HTWKPoint &positon, Orientation::MapTile orient)
            : id(id), tileTypeId(tileTypeId), neighbors(), positon(positon), orient(orient) {

    }

    MapElement(int id, MapTileType tileTypeId, const HTWKPoint &positon, Orientation::MapTile orient, float probability)
            : id(id), tileTypeId(tileTypeId), neighbors(), positon(positon), orient(orient), probability(probability) {
        //tileEntryPoints.reserve(4);
        //tileExitPoints.reserve(4);
    }

    const int getId() {
        return id;
    }

    const HTWKPoint &getPositon() const {
        return positon;
    }

    const Orientation::MapTile getOrient() const {
        return orient;
    }

    const float getProbability() const {
        return probability;
    }

    std::vector<TilePoint*> getTileEntryPoints() {
        return std::vector<TilePoint*>(std::begin(tileEntryPoints), std::end(tileEntryPoints));
    }

    std::vector<TilePoint*> getTileExitPoints() {
        return std::vector<TilePoint*>(std::begin(tileExitPoints), std::end(tileExitPoints));
    }

    const float &getTileSize() const {
        return tileSize;
    }

    const MapTileType &getTileTypeId() const {
        return tileTypeId;
    }

    void setNeighbor(const Orientation::Global &orient, MapElement *neighbor) {
        neighbors[static_cast<int>(orient)] = neighbor;
    }

    MapElement *getNeighbor(const Orientation::Global &orient) {
        return neighbors[static_cast<int>(orient)];
    }

    bool isSet() {
        return tileTypeId != MapTileType::NOT_SET;
    }

protected:
    virtual void generateTileEntryPoints() {};

    virtual void generateTileExitPoints() {};

private:
    int id = INVALID_TILE;
    MapTileType tileTypeId = MapTileType::NOT_SET;
    MapElement *neighbors[4]{nullptr, nullptr, nullptr, nullptr};

protected:
    HTWKPoint positon = HTWKPoint();
    Orientation::MapTile orient = Orientation::MapTile::NORTH;

    float probability = 1;
    float tileSize = 1;

    TilePoint* tileEntryPoints[4]{nullptr, nullptr, nullptr, nullptr};
    TilePoint* tileExitPoints[4]{nullptr, nullptr, nullptr, nullptr};
};

#pragma pack(pop)

#endif //AADC_USER_MAPELEMENT_H
