#ifndef _LANE_DATA_H_
#define _LANE_DATA_H_

#pragma pack(push, 1)
struct tLanePoint{
    tFloat xLeftLane = 0;
    tBool isLeftGuessed = tTrue;
    tFloat xRightLane = 0;
    tBool isRightGuessed = tTrue;
    tFloat y = 0;
};
#pragma pack(pop)

#define MEDIA_TYPE_LANE_DATA     0x00041677
#define MEDIA_SUBTYPE_LANE_DATA  0x00041677


#endif
