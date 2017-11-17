#ifndef AADC_USER_ROADSIGNEXT_H
#define AADC_USER_ROADSIGNEXT_H

#pragma pack(push, 1)
typedef struct {
    tInt16 i16Identifier;
    tFloat32 f32Imagesize;
    tFloat32 af32TVec[3];
    tFloat32 af32RVec[3];
} tRoadSignExt;
#pragma pack(pop)

#endif //AADC_USER_ROADSIGNEXT_H
