#include "MapDrawer.h"

void MapDrawer::drawMap(bool drawSigns) {
    std::string print;

    for (int y = bounds.maxY; y >= bounds.minY; y--) {
        printf("%*d │", 3, y);
        for (int x = bounds.minX; x <= bounds.maxX; x++) {
            if (tilesMap[x][y].isSet()) {
                auto tileType = tilesMap[x][y].getTileTypeId();
                switch (tileType) {
                    case MapTileType::INTERSECTION_PLUS:
                        print = " ╬";
                        break;
                    case MapTileType::INTERSECTION_T:
                        switch (tilesMap[x][y].getOrient()) {
                            case Orientation::MapTile::NORTH:
                                print = " ╩";
                                break;
                            case Orientation::MapTile::EAST:
                                print = " ╠";
                                break;
                            case Orientation::MapTile::SOUTH:
                                print = " ╦";
                                break;
                            case Orientation::MapTile::WEST:
                                print = " ╣";
                                break;
                        }
                        break;
                    case MapTileType::TURN_SMALL:
                        switch (tilesMap[x][y].getOrient()) {
                            case Orientation::MapTile::NORTH:
                                print = " ╚";
                                break;
                            case Orientation::MapTile::EAST:
                                print = " ╔";
                                break;
                            case Orientation::MapTile::SOUTH:
                                print = " ╗";
                                break;
                            case Orientation::MapTile::WEST:
                                print = " ╝";
                                break;
                        }
                        break;
                    case MapTileType::STRAIGHT:
                        switch (tilesMap[x][y].getOrient()) {
                            case Orientation::MapTile::NORTH:
                            case Orientation::MapTile::SOUTH:
                                print = " ║";
                                break;
                            case Orientation::MapTile::EAST:
                            case Orientation::MapTile::WEST:
                                print = " ═";
                                break;
                        }
                        break;

                    default:
                        print = mapTileTypeToString(tileType).substr(0, 1);
                        break;
                }
            }

                // signs
            else if (drawSigns && !signsMap[x][y].empty()) {
                std::string label = mapTypeToString(signsMap[x][y].front().type);
                if (label == "Pedestrian-Crossing")
                    label = "Zebra";
                print = label.substr(0, 1);
            } else {
                print = EMPTY_TILE;
            }

            printf("%*s ", 2, print.c_str());
        }
        printf("\n");
    }

    // x-axis legend
    for (int x = 0; x < (bounds.maxX - bounds.minX + 1) * 3 + 4; x++) {
        printf("─");
    }
    printf("\n     ");
    for (int x = bounds.minX; x <= bounds.maxX; x++) {
        printf("%*d ", 2, x);
    }
    printf("\n");
}