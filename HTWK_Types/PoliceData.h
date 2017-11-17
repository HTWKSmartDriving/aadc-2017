#ifndef AADC_USER_POLICE_DATA_H
#define AADC_USER_POLICE_DATA_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

//KuerData
#pragma pack(push, 1)
struct tPoliceData {
    tUInt64 countSeen = 0; // 1,2,Polizei,3,4,Offizier,5,6,alte Hex,.. *sing*
    tTimeStamp lastSeen = 0;
};
#pragma pack(pop)

#define MEDIA_TYPE_POLICE_DATA     0x02131629
#define MEDIA_SUBTYPE_POLICE_DATA  0x02131629

#endif
