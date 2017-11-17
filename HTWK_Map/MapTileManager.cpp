//
// Created by fabian on 18.10.17.
//

#include "MapTileManager.h"

MapTileManager::MapTileManager() {
    mapTileCounter = 0;
}

MapTileManager::~MapTileManager() {
    for(auto& element: elements) {
        for(auto& point : element.getTileEntryPoints()) {
            if(point != nullptr) {
                delete(point);
            }
        }

        for(auto& point : element.getTileExitPoints()) {
            if(point != nullptr) {
                delete(point);
            }
        }
    }
}

void MapTileManager::registerMapTile(MapElement mapTile) {
    elements.push_back(mapTile);
}

void MapTileManager::registerMapVector(std::vector<MapElement> elements) {
    for(auto& element: elements) {
        registerMapTile(element);
    }
}
