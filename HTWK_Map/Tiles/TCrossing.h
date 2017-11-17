//
// Created by fabian on 11.08.17.
//

#ifndef AADC_USER_TCROSSING_H
#define AADC_USER_TCROSSING_H

#include "../MapElement.h"

#define T_CROSSING_TILE_SIZE 1

class TCrossing : public MapElement {
public:
    TCrossing(int id, MapTileType tileTypeId, const HTWKPoint &positon, Orientation::MapTile orient)
            : MapElement(id, tileTypeId, positon, orient) {
        tileSize = T_CROSSING_TILE_SIZE;
        generateTileEntryPoints();
        generateTileExitPoints();

        //set next und last pointer of connected entry and exit points
        tileEntryPoints[0]->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)] = tileExitPoints[1];
        tileExitPoints[1]->last[static_cast<int>(Orientation::TurnDirection::STRAIGHT)] = tileEntryPoints[0];
        tileEntryPoints[0]->next[static_cast<int>(Orientation::TurnDirection::LEFT)] = tileExitPoints[2];
        tileExitPoints[2]->last[static_cast<int>(Orientation::TurnDirection::LEFT)] = tileEntryPoints[0];

        tileEntryPoints[1]->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)] = tileExitPoints[0];
        tileExitPoints[0]->last[static_cast<int>(Orientation::TurnDirection::STRAIGHT)] = tileEntryPoints[1];
        tileEntryPoints[1]->next[static_cast<int>(Orientation::TurnDirection::RIGHT)] = tileExitPoints[2];
        tileExitPoints[2]->last[static_cast<int>(Orientation::TurnDirection::RIGHT)] = tileEntryPoints[1];

        tileEntryPoints[2]->next[static_cast<int>(Orientation::TurnDirection::RIGHT)] = tileExitPoints[0];
        tileExitPoints[0]->last[static_cast<int>(Orientation::TurnDirection::RIGHT)] = tileEntryPoints[2];
        tileEntryPoints[2]->next[static_cast<int>(Orientation::TurnDirection::LEFT)] = tileExitPoints[1];
        tileExitPoints[1]->last[static_cast<int>(Orientation::TurnDirection::LEFT)] = tileEntryPoints[2];
    }

private:
    void generateTileEntryPoints() override {
        TilePoint tileEntryPoint1, tileEntryPoint2, tileEntryPoint3;

        switch (orient) {
            case Orientation::MapTile::NORTH:
                tileEntryPoint1.setXval(
                        positon.x() + SPACE_NEXT_TO_SIDE_LINE_WIDTH + SIDE_LINE_WIDTH + TRACK_WIDTH +
                        MIDDLE_LINE_WIDTH + TRACK_WIDTH / 2);
                tileEntryPoint1.setYval(positon.y());
                tileEntryPoint1.edge = Orientation::Global::SOUTH;
                tileEntryPoint2.setXval(
                        positon.x() + SPACE_NEXT_TO_SIDE_LINE_WIDTH + SIDE_LINE_WIDTH + TRACK_WIDTH / 2);
                tileEntryPoint2.setYval(positon.y() + tileSize);
                tileEntryPoint2.edge = Orientation::Global::NORTH;
                tileEntryPoint3.setXval(positon.x());
                tileEntryPoint3.setYval(
                        positon.y() + SPACE_NEXT_TO_SIDE_LINE_WIDTH + SIDE_LINE_WIDTH + TRACK_WIDTH / 2);
                tileEntryPoint3.edge = Orientation::Global::WEST;
                break;
            case Orientation::MapTile::WEST:
                tileEntryPoint1.setXval(positon.x() + tileSize);
                tileEntryPoint1.setYval(
                        positon.y() + SPACE_NEXT_TO_SIDE_LINE_WIDTH + SIDE_LINE_WIDTH + TRACK_WIDTH +
                        MIDDLE_LINE_WIDTH + TRACK_WIDTH / 2);
                tileEntryPoint1.edge = Orientation::Global::EAST;
                tileEntryPoint2.setXval(positon.x());
                tileEntryPoint2.setYval(
                        positon.y() + SPACE_NEXT_TO_SIDE_LINE_WIDTH + SIDE_LINE_WIDTH + TRACK_WIDTH / 2);
                tileEntryPoint2.edge = Orientation::Global::WEST;
                tileEntryPoint3.setXval(
                        positon.x() + SPACE_NEXT_TO_SIDE_LINE_WIDTH + SIDE_LINE_WIDTH + TRACK_WIDTH +
                        MIDDLE_LINE_WIDTH + TRACK_WIDTH / 2);
                tileEntryPoint3.setYval(positon.y());
                tileEntryPoint3.edge = Orientation::Global::SOUTH;
                break;
            case Orientation::MapTile::EAST:
                tileEntryPoint1.setXval(positon.x());
                tileEntryPoint1.setYval(
                        positon.y() + SPACE_NEXT_TO_SIDE_LINE_WIDTH + SIDE_LINE_WIDTH + TRACK_WIDTH / 2);
                tileEntryPoint1.edge = Orientation::Global::WEST;
                tileEntryPoint2.setXval(positon.x() + tileSize);
                tileEntryPoint2.setYval(
                        positon.y() + SPACE_NEXT_TO_SIDE_LINE_WIDTH + SIDE_LINE_WIDTH + TRACK_WIDTH +
                        MIDDLE_LINE_WIDTH + TRACK_WIDTH / 2);
                tileEntryPoint2.edge = Orientation::Global::EAST;
                tileEntryPoint3.setXval(
                        positon.x() + SPACE_NEXT_TO_SIDE_LINE_WIDTH + SIDE_LINE_WIDTH + TRACK_WIDTH / 2);
                tileEntryPoint3.setYval(positon.y() + tileSize);
                tileEntryPoint3.edge = Orientation::Global::NORTH;

                break;
            case Orientation::MapTile::SOUTH:
                tileEntryPoint1.setXval(
                        positon.x() + SPACE_NEXT_TO_SIDE_LINE_WIDTH + SIDE_LINE_WIDTH + TRACK_WIDTH / 2);
                tileEntryPoint1.setYval(positon.y() + tileSize);
                tileEntryPoint1.edge = Orientation::Global::NORTH;
                tileEntryPoint2.setXval(
                        positon.x() + SPACE_NEXT_TO_SIDE_LINE_WIDTH + SIDE_LINE_WIDTH + TRACK_WIDTH +
                        MIDDLE_LINE_WIDTH + TRACK_WIDTH / 2);
                tileEntryPoint2.setYval(positon.y());
                tileEntryPoint2.edge = Orientation::Global::SOUTH;
                tileEntryPoint3.setXval(positon.x() + tileSize);
                tileEntryPoint3.setYval(
                        positon.y() + SPACE_NEXT_TO_SIDE_LINE_WIDTH + SIDE_LINE_WIDTH + TRACK_WIDTH +
                        MIDDLE_LINE_WIDTH + TRACK_WIDTH / 2);
                tileEntryPoint3.edge = Orientation::Global::EAST;
                break;
        }

        tileEntryPoints[0] = new TilePoint(tileEntryPoint1);
        tileEntryPoints[1] = new TilePoint(tileEntryPoint2);
        tileEntryPoints[2] = new TilePoint(tileEntryPoint3);
    }

protected:
    void generateTileExitPoints() override {
        TilePoint tileExitPoint1, tileExitPoint2, tileExitPoint3;

        tileExitPoint1.edge = tileEntryPoints[0]->edge;
        tileExitPoint2.edge = tileEntryPoints[1]->edge;
        tileExitPoint3.edge = tileEntryPoints[2]->edge;

        switch (orient) {
            case Orientation::MapTile::NORTH:
                tileExitPoint1.setXval(tileEntryPoints[1]->x());
                tileExitPoint1.setYval(tileEntryPoints[1]->y() - T_CROSSING_TILE_SIZE);
                tileExitPoint2.setXval(tileEntryPoints[0]->x());
                tileExitPoint2.setYval(tileEntryPoints[0]->y() + T_CROSSING_TILE_SIZE);
                tileExitPoint3.setXval(tileEntryPoints[2]->x());
                tileExitPoint3.setYval(tileEntryPoints[2]->y() + TRACK_WIDTH / 2 + MIDDLE_LINE_WIDTH + TRACK_WIDTH / 2);
                break;
            case Orientation::MapTile::WEST:
                tileExitPoint1.setXval(tileEntryPoints[1]->x() + T_CROSSING_TILE_SIZE);
                tileExitPoint1.setYval(tileEntryPoints[1]->y());
                tileExitPoint2.setXval(tileEntryPoints[0]->x() - T_CROSSING_TILE_SIZE);
                tileExitPoint2.setYval(tileEntryPoints[0]->y());
                tileExitPoint3.setXval(
                        tileEntryPoints[2]->x() - (TRACK_WIDTH / 2 + MIDDLE_LINE_WIDTH + TRACK_WIDTH / 2));
                tileExitPoint3.setYval(tileEntryPoints[2]->y());
                break;
            case Orientation::MapTile::EAST:
                tileExitPoint1.setXval(tileEntryPoints[1]->x() - T_CROSSING_TILE_SIZE);
                tileExitPoint1.setYval(tileEntryPoints[1]->y());
                tileExitPoint2.setXval(tileEntryPoints[0]->x() + T_CROSSING_TILE_SIZE);
                tileExitPoint2.setYval(tileEntryPoints[0]->y());
                tileExitPoint3.setXval(tileEntryPoints[2]->x() + TRACK_WIDTH / 2 + MIDDLE_LINE_WIDTH + TRACK_WIDTH / 2);
                tileExitPoint3.setYval(tileEntryPoints[2]->y());
                break;
            case Orientation::MapTile::SOUTH:
                tileExitPoint1.setXval(tileEntryPoints[1]->x());
                tileExitPoint1.setYval(tileEntryPoints[1]->y() + T_CROSSING_TILE_SIZE);
                tileExitPoint2.setXval(tileEntryPoints[0]->x());
                tileExitPoint2.setYval(tileEntryPoints[0]->y() - T_CROSSING_TILE_SIZE);
                tileExitPoint3.setXval(tileEntryPoints[2]->x());
                tileExitPoint3.setYval(
                        tileEntryPoints[2]->y() - (TRACK_WIDTH / 2 + MIDDLE_LINE_WIDTH + TRACK_WIDTH / 2));
                break;
        }

        tileExitPoints[0] = new TilePoint(tileExitPoint1);
        tileExitPoints[1] = new TilePoint(tileExitPoint2);
        tileExitPoints[2] = new TilePoint(tileExitPoint3);
    }
};


#endif //AADC_USER_TCROSSING_H
