//
// Created by fabian on 18.10.17.
//

#ifndef AADC_USER_MAPTILEMANAGER_H
#define AADC_USER_MAPTILEMANAGER_H

#include "MapElement.h"

class MapTileManager {
private:
    int mapTileCounter;
    std::vector<MapElement> elements;

public:
    MapTileManager();

    virtual ~MapTileManager();

    void registerMapTile(MapElement mapTile);

    void registerMapVector(std::vector<MapElement> elements);
};


#endif //AADC_USER_MAPTILEMANAGER_H
