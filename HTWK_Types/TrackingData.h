#ifndef AADC_USER_TRACKINGDATA_H
#define AADC_USER_TRACKINGDATA_H

#include "ObstacleData.h"

#pragma pack(push, 1)
struct tTrackingData {
    float radius = 0;
    cv::Point2f position = cv::Point2f(0, 0);
    ObstacleStatus status = ObstacleStatus::UNKNOWN;
    ObstacleType type = ObstacleType::NONE;
    bool newlyTracked = true;
};
#pragma pack(pop)

#define MEDIA_TYPE_TRACKING_DATA     0x05141623
#define MEDIA_SUBTYPE_TRACKING_DATA  0x05141623

#endif
