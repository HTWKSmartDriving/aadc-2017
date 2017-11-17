#ifndef _CAMERA_RESOLUTION_DATA_H_
#define _CAMERA_RESOLUTION_DATA_H_

#pragma pack(push, 1)
struct tCameraResolutionData{
    tInt32 height = 0;
    tInt32 width = 0;
};
#pragma pack(pop)

#define MEDIA_TYPE_CAMERA_RESOLUTION_DATA     0x00041678
#define MEDIA_SUBTYPE_CAMERA_RESOLUTION_DATA  0x00041678

#endif
