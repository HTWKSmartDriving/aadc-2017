//
// Created by fabian on 16.09.17.
//

#ifndef AADC_USER_MAPCHANGE_H
#define AADC_USER_MAPCHANGE_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

struct tMapChange {
    tInt16 i32x;
    tInt16 i32y;
    tInt16 i16orientation;
    tInt8 i8tileId;
};

#endif //AADC_USER_MAPCHANGE_H
