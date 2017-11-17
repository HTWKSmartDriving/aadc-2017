#ifndef _ROAD_SIGN_DATA_H_
#define _ROAD_SIGN_DATA_H_

#include <opencv2/opencv.hpp>
#include "RoadSignEnum.h"

#pragma pack(push, 1)
struct tRoadSignDetectionResult {
    ROAD_SIGN id = ROAD_SIGN::UNMARKED;
    tFloat32 imagesize = 0;
    cv::Point3f tVec = cv::Point3f();
    cv::Point3f rVec = cv::Point3f();
};
#pragma pack(pop)

#define MEDIA_TYPE_ROAD_SIGN_DATA     0x01241677
#define MEDIA_SUBTYPE_ROAD_SIGN_DATA  0x01241677

#endif
