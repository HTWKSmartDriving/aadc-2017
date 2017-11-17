#ifndef AADC_USER_HTWKMAP_H
#define AADC_USER_HTWKMAP_H

#include "stdafx.h"
#include "tinyxml2.h"
#include "../HTWK_Utils/HTWKPoint.hpp"
#include "../HTWK_Types/MapTileType.h"
#include "Orientation.h"
#include "MapElement.h"
#include <iostream>
#include "../HTWK_Utils/HTWKUtils.hpp"
#include "../HTWK_Utils/HTWKMathUtils.hpp"

#include "Tiles/Tiles.h"
#include "../../HTWK_Debug/EnableLogs.h"
#define DIMENSION_DEFAULT 1.0f

class TrackMap {
private:
    cFilename file;
    int idIncrementor = 0;
    std::vector<MapElement> mapElements;

public:
    TrackMap();

    explicit TrackMap(std::vector<MapElement> mapElements);

    tResult readXMLMap();

    void setFile(const cFilename &file);

    int getHeightInMeters();

    int getWidthInMeters();

    int getMinX();

    int getMaxX();

    int getMinY();

    int getMaxY();

    MapElement *getTileByPosition(const HTWKPoint &position);

    TilePoint *getTilePointByPosition(const HTWKPoint &position, const float &orientationInRad);

    void
    getTrajectoryBetweenPoints(TilePoint *tp1, TilePoint *tp2, const float &stepSize, std::vector<HTWKPoint> &points);

    void insertMapTile(MapElement &element);

    void setTileLinks();

    int getIdIncrementor();

    const MapElement &getTileByTilePoint(const TilePoint *tp);

    TilePoint *getTilePointOppositeLane(TilePoint *pPoint);

    void clear();

    void push (const MapElement& element);

    std::vector<MapElement> asVector ();

    bool empty ();

};


#endif //AADC_USER_HTWKMAP_H
