#ifndef AADC_USER_STIXELDATA_H
#define AADC_USER_STIXELDATA_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

#pragma pack(push, 1)
struct tStixel {
    tInt column = 0;
    tInt rowTop = 0;
    tInt rowBottom = 0;
    tFloat disparity = 0;
};
#pragma pack(pop)

#define MEDIA_TYPE_STIXEL_DATA     0x00042677
#define MEDIA_SUBTYPE_STIXEL_DATA  0x00042677

#endif //AADC_USER_STIXELDATA_H
