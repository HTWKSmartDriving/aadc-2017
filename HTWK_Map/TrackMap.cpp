#include "TrackMap.h"

TrackMap::TrackMap() = default;

TrackMap::TrackMap(std::vector<MapElement> mapElements) {
    this->mapElements = std::move(mapElements);
}

tResult TrackMap::readXMLMap() {
    // Get path of configuration file
    cFilename fileMap = this->file;
    ADTF_GET_CONFIG_FILENAME(fileMap);
    fileMap = fileMap.CreateAbsolutePath(".");

    // check if given property is not empty
    if (fileMap.IsEmpty()) {
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    if (cFileSystem::Exists(fileMap)) {
        tinyxml2::XMLDocument xmlDocument;

        tinyxml2::XMLError eResult = xmlDocument.LoadFile(fileMap);

        if (!(eResult == tinyxml2::XMLError::XML_SUCCESS)) {
            LOG_ERROR("Can't open File");
            RETURN_ERROR(ERR_INVALID_FILE);
        }

        tinyxml2::XMLNode *pRoot = xmlDocument.FirstChild();

        //check root node
        if (pRoot == nullptr) {
            LOG_ERROR("No Root Element found");
            RETURN_ERROR(ERR_INVALID_FILE);
        }

        for (tinyxml2::XMLElement *xmlTileElement = pRoot->NextSibling()->FirstChildElement();
             xmlTileElement != nullptr; xmlTileElement = xmlTileElement->NextSiblingElement()) {
            tinyxml2::XMLElement *tmpXmlTileElement = xmlTileElement;

            if (std::strcmp(tmpXmlTileElement->Value(), "tile") == 0) {
                MapTileType tileId;
                double x = 0;
                double y = 0;
                Orientation::MapTile orientation;

                tileId = static_cast<MapTileType>(tmpXmlTileElement->IntAttribute("id"));
                tmpXmlTileElement->QueryDoubleAttribute("x", &x);
                tmpXmlTileElement->QueryDoubleAttribute("y", &y);
                orientation = static_cast<Orientation::MapTile>(tmpXmlTileElement->IntAttribute("direction"));

                unique_ptr<MapElement> me;

                switch (tileId) {
                    case MapTileType::STRAIGHT:
                        me.reset(new Straight(idIncrementor++, tileId, HTWKPoint(x, y), orientation));
                        break;
                    case MapTileType::INTERSECTION_T:
                        me.reset(new TCrossing(idIncrementor++, tileId, HTWKPoint(x, y), orientation));
                        break;
                    case MapTileType::INTERSECTION_PLUS:
                        me.reset(new PlusCrossing(idIncrementor++, tileId, HTWKPoint(x, y), orientation));
                        break;
                    case MapTileType::TURN_LARGE:
                        me.reset(new LargeTurn(idIncrementor++, tileId, HTWKPoint(x, y), orientation));
                        break;
                    case MapTileType::TURN_SMALL:
                        me.reset(new SmallTurn(idIncrementor++, tileId, HTWKPoint(x, y), orientation));
                        break;
                    case MapTileType::LOT:
                        break;
                    case MapTileType::TURN_S_LEFT:
                        me.reset(new STurnLeft(idIncrementor++, tileId, HTWKPoint(x, y), orientation));
                        break;
                    case MapTileType::TURN_S_RIGHT:
                        me.reset(new STurnRight(idIncrementor++, tileId, HTWKPoint(x, y), orientation));
                        break;
                }

                insertMapTile(*me);
            }
        }

    }

    RETURN_NOERROR;
}

void TrackMap::insertMapTile(MapElement &element) {
    mapElements.emplace_back(element);
}

int TrackMap::getIdIncrementor() {
    return idIncrementor++;
}

void TrackMap::setTileLinks() {
    //set neighbor pointer
    for (auto &element: mapElements) {
        for (auto &pointEntry: element.getTileEntryPoints()) {
            if (pointEntry != nullptr) {
                HTWKPoint pointOnNeighborTile;

                switch (pointEntry->edge) {
                    case Orientation::Global::NORTH:
                        pointOnNeighborTile = pointEntry->moveNewPointByXY(0, DIMENSION_DEFAULT / 2);
                        break;
                    case Orientation::Global::WEST:
                        pointOnNeighborTile = pointEntry->moveNewPointByXY(-DIMENSION_DEFAULT / 2, 0);
                        break;
                    case Orientation::Global::EAST:
                        pointOnNeighborTile = pointEntry->moveNewPointByXY(DIMENSION_DEFAULT / 2, 0);
                        break;
                    case Orientation::Global::SOUTH:
                        pointOnNeighborTile = pointEntry->moveNewPointByXY(0, -DIMENSION_DEFAULT / 2);
                        break;
                }

                element.setNeighbor(pointEntry->edge, getTileByPosition(pointOnNeighborTile));

                //link all tile entry points together
                if (element.getNeighbor(pointEntry->edge) != nullptr) {
                    for (auto &pointExitNeighbor: element.getNeighbor(pointEntry->edge)->getTileExitPoints()) {
                        if (pointExitNeighbor != nullptr) {
                            if (HTWKUtils::Equals(*pointEntry, *pointExitNeighbor, 0.10)) {
                                for (unsigned int i = 0; i < 3; i++) {
                                    if (pointExitNeighbor->last[i] != nullptr) {
                                        pointExitNeighbor->last[i]->next[i] = pointEntry;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void TrackMap::setFile(const cFilename &file) {
    TrackMap::file = file;
}

int TrackMap::getHeightInMeters() {
    return !mapElements.empty() ? (getMaxY() - getMinY()) : 0;
}

int TrackMap::getWidthInMeters() {
    return !mapElements.empty() ? (getMaxX() - getMinX()) : 0;
}

int TrackMap::getMinX() {
    double minX = std::numeric_limits<double>::max();

    for (auto const &value: mapElements) {
        if (minX > value.getPositon().x())
            minX = value.getPositon().x();
    }

    return static_cast<int>(minX);
}

int TrackMap::getMaxX() {
    double maxX = std::numeric_limits<double>::lowest();
    float tileCountCorrection = 0;

    for (auto const &value: mapElements) {
        tileCountCorrection = value.getTileSize();

        if (maxX < (value.getPositon().x() + tileCountCorrection))
            maxX = value.getPositon().x() + tileCountCorrection;
    }

    return static_cast<int>(maxX);
}

int TrackMap::getMinY() {
    double minY = std::numeric_limits<double>::max();

    for (auto const &value: mapElements) {
        if (minY > value.getPositon().y())
            minY = value.getPositon().y();
    }

    return static_cast<int>(minY);
}

int TrackMap::getMaxY() {
    double maxY = std::numeric_limits<double>::lowest();
    float tileCountCorrection = 0;

    for (auto const &value: mapElements) {
        tileCountCorrection = value.getTileSize();

        if (maxY < (value.getPositon().y() + tileCountCorrection))
            maxY = value.getPositon().y() + tileCountCorrection;
    }

    return static_cast<int>(maxY);
}

MapElement *TrackMap::getTileByPosition(const HTWKPoint &position) {
    for (auto &tile: mapElements) {
        if (position.x() >= tile.getPositon().x() && position.x() <= (tile.getPositon().x() + tile.getTileSize()) &&
            position.y() >= tile.getPositon().y() && position.y() <= (tile.getPositon().y() + tile.getTileSize())) {
            return &tile;
        }
    }

    return nullptr;
}

TilePoint *TrackMap::getTilePointByPosition(const HTWKPoint &position, const float &orientationInRad) {
    MapElement *mapElementPtr = getTileByPosition(position);

    if (mapElementPtr != nullptr) {
        switch (mapElementPtr->getOrient()) {
            case Orientation::MapTile::NORTH:
                if (orientationInRad > 0 && orientationInRad < PI)
                    return mapElementPtr->getTileEntryPoints().at(0);
                else
                    return mapElementPtr->getTileEntryPoints().at(1);
            case Orientation::MapTile::WEST:
                if (orientationInRad > PI / 2 || orientationInRad < -PI / 2)
                    return mapElementPtr->getTileEntryPoints().at(0);
                else
                    return mapElementPtr->getTileEntryPoints().at(1);
            case Orientation::MapTile::EAST:
                if (orientationInRad > -PI / 2 && orientationInRad < PI / 2)
                    return mapElementPtr->getTileEntryPoints().at(0);
                else
                    return mapElementPtr->getTileEntryPoints().at(1);
            case Orientation::MapTile::SOUTH:
                if (orientationInRad > -PI && orientationInRad < 0)
                    return mapElementPtr->getTileEntryPoints().at(0);
                else
                    return mapElementPtr->getTileEntryPoints().at(1);
        }
    }

    return nullptr;
}

const MapElement &TrackMap::getTileByTilePoint(const TilePoint *tp) {
    if (tp != nullptr) {
        for (auto &tile: mapElements) {
            for (auto &exitPoint: tile.getTileExitPoints()) {
                if (exitPoint != nullptr) {
                    if (tp == exitPoint) {
                        return tile;
                    }
                }

            }

            for (auto &entryPoint: tile.getTileEntryPoints()) {
                if (entryPoint != nullptr) {
                    if (tp == entryPoint) {
                        return tile;
                    }
                }
            }
        }
    }
    LOG_ERROR("tile point error");
    return mapElements.at(0);
}


TilePoint *TrackMap::getTilePointOppositeLane(TilePoint *pPoint) {
    MapElement element = getTileByTilePoint(pPoint);
    for (const auto &tilePoint: element.getTileEntryPoints()) {
        if (tilePoint != nullptr && tilePoint != pPoint) {
            return tilePoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)];
        }
    }

    return nullptr;
}

void TrackMap::getTrajectoryBetweenPoints(TilePoint *tp1, TilePoint *tp2, const float &stepSize,
                                          std::vector<HTWKPoint> &points) {
    float sampleValue = 0;
    double distance = tp1->dist(*tp2);

    if (HTWKUtils::Equals(tp1->x(), tp2->x(), 0.05)) {
        //straight-path in x-direction
        if (tp1->y() > tp2->y()) {
            while (sampleValue < distance) {
                points.emplace_back(HTWKPoint(tp1->x(), tp1->y() - sampleValue));

                sampleValue += stepSize;
            }
        } else {
            while (sampleValue < distance) {
                points.emplace_back(HTWKPoint(tp2->x(), tp2->y() - sampleValue));

                sampleValue += stepSize;
            }
        }

    } else if (HTWKUtils::Equals(tp1->y(), tp2->y(), 0.05)) {
        //straight-path in y-direction
        if (tp1->x() > tp2->x()) {
            while (sampleValue < distance) {
                points.emplace_back(HTWKPoint(tp1->x() - sampleValue, tp1->y()));

                sampleValue += stepSize;
            }
        } else {
            while (sampleValue < distance) {
                points.emplace_back(HTWKPoint(tp2->x() - sampleValue, tp2->y()));

                sampleValue += stepSize;
            }
        }

    } else if (tp1->edge == tp2->edge) {
        //s-turn
        HTWKPoint center1, center2;
        double sampleCorrection1, sampleCorrection2;

        //TODO Fix MagicNumbers
        switch (tp1->edge) {
            case Orientation::Global::NORTH:
                if (tp2->x() < tp1->x()) {
                    //s right
                    center2.setXval(std::floor(tp2->x()) + STURN_TILE_SIZE - STURN_CIRCLE_CENTER_OFFSET);
                    center2.setYval(tp2->y());
                    center1.setXval(std::ceil(tp1->x()) - STURN_TILE_SIZE + STURN_CIRCLE_CENTER_OFFSET);
                    center1.setYval(tp1->y());


                    sampleCorrection1 = -PI / 2 + 28 * PI / 180;
                    sampleCorrection2 = PI / 2 + 28 * PI / 180;
                } else {
                    //s left
                    center1.setXval(std::floor(tp1->x()) + STURN_TILE_SIZE - STURN_CIRCLE_CENTER_OFFSET);
                    center1.setYval(tp1->y());
                    center2.setXval(std::ceil(tp2->x()) - STURN_TILE_SIZE + STURN_CIRCLE_CENTER_OFFSET);
                    center2.setYval(tp2->y());

                    sampleCorrection1 = PI;
                    sampleCorrection2 = 0;
                }
                break;
            case Orientation::Global::WEST:
                if (tp2->y() < tp1->y()) {
                    //s right
                    center2.setXval(tp2->x());
                    center2.setYval(std::floor(tp2->y()) + STURN_TILE_SIZE - STURN_CIRCLE_CENTER_OFFSET);
                    center1.setXval(tp1->x());
                    center1.setYval(std::ceil(tp1->y()) - STURN_TILE_SIZE + STURN_CIRCLE_CENTER_OFFSET);

                    sampleCorrection1 = 0 + 28 * PI / 180;
                    sampleCorrection2 = PI + 28 * PI / 180;
                } else {
                    //s left
                    center1.setXval(tp1->x());
                    center1.setYval(std::floor(tp1->y()) + STURN_TILE_SIZE - STURN_CIRCLE_CENTER_OFFSET);
                    center2.setXval(tp2->x());
                    center2.setYval(std::ceil(tp2->y()) - STURN_TILE_SIZE + STURN_CIRCLE_CENTER_OFFSET);

                    sampleCorrection1 = -PI / 2;
                    sampleCorrection2 = PI / 2;
                }
                break;
            case Orientation::Global::EAST:
                if (tp2->y() < tp1->y()) {
                    //s left
                    center2.setXval(tp2->x());
                    center2.setYval(std::floor(tp2->y()) + STURN_TILE_SIZE - STURN_CIRCLE_CENTER_OFFSET);
                    center1.setXval(tp1->x());
                    center1.setYval(std::ceil(tp1->y()) - STURN_TILE_SIZE + STURN_CIRCLE_CENTER_OFFSET);

                    sampleCorrection1 = PI / 2;
                    sampleCorrection2 = -PI / 2;
                } else {
                    //s right
                    center1.setXval(tp1->x());
                    center1.setYval(std::floor(tp1->y()) + STURN_TILE_SIZE - STURN_CIRCLE_CENTER_OFFSET);
                    center2.setXval(tp2->x());
                    center2.setYval(std::ceil(tp2->y()) - STURN_TILE_SIZE + STURN_CIRCLE_CENTER_OFFSET);

                    sampleCorrection1 = PI + 28 * PI / 180;
                    sampleCorrection2 = 0 + 28 * PI / 180;
                }
                break;
            case Orientation::Global::SOUTH:
                if (tp2->x() < tp1->x()) {
                    //s left
                    center2.setXval(std::floor(tp2->x()) + STURN_TILE_SIZE - STURN_CIRCLE_CENTER_OFFSET);
                    center2.setYval(tp2->y());
                    center1.setXval(std::ceil(tp1->x()) - STURN_TILE_SIZE + STURN_CIRCLE_CENTER_OFFSET);
                    center1.setYval(tp1->y());

                    sampleCorrection1 = 0;
                    sampleCorrection2 = PI;
                } else {
                    //s right
                    center1.setXval(std::floor(tp1->x()) + STURN_TILE_SIZE - STURN_CIRCLE_CENTER_OFFSET);
                    center1.setYval(tp1->y());
                    center2.setXval(std::ceil(tp2->x()) - STURN_TILE_SIZE + STURN_CIRCLE_CENTER_OFFSET);
                    center2.setYval(tp2->y());

                    sampleCorrection1 = PI / 2 + 28 * PI / 180;
                    sampleCorrection2 = -PI / 2 + 28 * PI / 180;
                }
                break;
        }

        double centerAngle = 0;
        double radius = center1.dist(*tp1);

        while (std::abs(centerAngle) < (62 * PI / 180)) {
            centerAngle = sampleValue / radius;

            HTWKPoint newPoint = center1.moveNewPointByDistanceAndAngleRad(radius, centerAngle + sampleCorrection1);
            points.emplace_back(newPoint);

            sampleValue += stepSize;
        }

        centerAngle = 0;
        sampleValue = 0;
        radius = center2.dist(*tp2);

        while (std::abs(centerAngle) < (62 * PI / 180)) {
            centerAngle = sampleValue / radius;

            HTWKPoint newPoint = center2.moveNewPointByDistanceAndAngleRad(radius, centerAngle + sampleCorrection2);
            points.emplace_back(newPoint);

            sampleValue += stepSize;
        }


    } else {
        //circle-path
        HTWKPoint center;

        switch (tp1->edge) {
            case Orientation::Global::NORTH:
                center.setXval(tp2->x());
                center.setYval(tp1->y());
                break;
            case Orientation::Global::WEST:
                center.setXval(tp1->x());
                center.setYval(tp2->y());
                break;
            case Orientation::Global::EAST:
                center.setXval(tp1->x());
                center.setYval(tp2->y());
                break;
            case Orientation::Global::SOUTH:
                center.setXval(tp2->x());
                center.setYval(tp1->y());
                break;
        }

        double sampleCorrection = 0;

        if (static_cast<Orientation::Global>((static_cast<int>(tp1->edge) + 1) % 4) == tp2->edge) {
            //rechtskurve
            switch (tp1->edge) {
                case Orientation::Global::NORTH:
                    sampleCorrection = PI;
                    break;
                case Orientation::Global::WEST:
                    sampleCorrection = -PI / 2;
                    break;
                case Orientation::Global::EAST:
                    sampleCorrection = PI / 2;
                    break;
                case Orientation::Global::SOUTH:
                    sampleCorrection = 0;
                    break;
            }
        } else {
            //linkskurve
            switch (tp1->edge) {
                case Orientation::Global::NORTH:
                    sampleCorrection = -PI / 2;
                    break;
                case Orientation::Global::WEST:
                    sampleCorrection = 0;
                    break;
                case Orientation::Global::EAST:
                    sampleCorrection = PI;
                    break;
                case Orientation::Global::SOUTH:
                    sampleCorrection = PI / 2;
                    break;
            }
        }

        double centerAngle = 0;
        double radius = center.dist(*tp1);

        while (std::abs(centerAngle) < PI / 2) {
            centerAngle = sampleValue / radius;

            HTWKPoint newPoint = center.moveNewPointByDistanceAndAngleRad(radius, centerAngle + sampleCorrection);
            points.emplace_back(newPoint);

            sampleValue += stepSize;
        }

/*
        //linkskurve
        if(static_cast<Orientation::Global>((static_cast<int>(tp1->edge) + 1) % 4) == tp2->edge)
        {
            switch(tp1->edge)
            {
                case Orientation::Global::NORTH:
                    center.setXval(tp2->x());
                    center.setYval(tp1->y());
                    break;
                case Orientation::Global::WEST:
                    center.setXval(tp1->x());
                    center.setYval(tp2->y());
                    break;
                case Orientation::Global::EAST:
                    center.setXval(tp1->x());
                    center.setYval(tp2->y());
                    break;
                case Orientation::Global::SOUTH:
                    center.setXval(tp2->x());
                    center.setYval(tp1->y());
                    break;
            }
        }

        //rechtskurve
        if(static_cast<Orientation::Global>((static_cast<int>(tp1->edge) - 1) % 4) == tp2->edge)
        {

        }*/
    }



}

void TrackMap::clear() {
    mapElements.clear();
}

void TrackMap::push(const MapElement &element) {
    mapElements.emplace_back(element);
}

std::vector<MapElement> TrackMap::asVector() {
    return mapElements;
}

bool TrackMap::empty() {
    return mapElements.empty();
}
