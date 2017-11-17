#ifndef AADC_USER_PARKINGLOT_H
#define AADC_USER_PARKINGLOT_H

#include "../HTWK_Map/Orientation.h"
#include "../HTWK_Utils/HTWKPoint.hpp"

#pragma pack(push, 1)
struct ParkingLot {
    int id;
    HTWKPoint position;
    int status;
    Orientation::MapTile direction;
};
#pragma pack(pop)

#endif //AADC_USER_PARKINGLOT_H
