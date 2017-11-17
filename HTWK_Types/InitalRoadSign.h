#ifndef INITAL_ROAD_SIGN_DATA_H_
#define INITAL_ROAD_SIGN_DATA_H_

#include <opencv2/opencv.hpp>
#include "RoadSignEnum.h"

#pragma pack(push, 1)
struct tInitialRoadSign {
    ROAD_SIGN id = ROAD_SIGN::UNMARKED;
    bool init = false;
    cv::Point2f pos = cv::Point2f(0, 0);
    float radius = 0;
    float heading = 0;
};
#pragma pack(pop)

#define MEDIA_TYPE_INITAL_ROAD_SIGN_DATA     0x01341677
#define MEDIA_SUBTYPE_INITAL_ROAD_SIGN_DATA  0x01341677

#endif
