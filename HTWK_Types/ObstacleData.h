#ifndef AADC_USER_OBSTACLEDATA_H
#define AADC_USER_OBSTACLEDATA_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <opencv2/opencv.hpp>

enum ObstacleType {
    NONE = 0,
    CAR = 1,
    ADULT = 2,
    CHILD = 3,
    SIGN = 4,
    PYLON = 5,
    ROAD = 6
};

static const std::map<const ObstacleType, const std::string> OBSTACLE_TYPE_TO_NAME_MAP = {
        {NONE, "none"},
        {CAR, "car"},
        {ADULT, "adult"},
        {CHILD, "child"},
        {SIGN, "sign"},
        {PYLON, "pylon"},
        {ROAD, "road"}
};
static const std::map<const std::string, const ObstacleType> OBSTACLE_NAME_TO_TYPE_MAP = {
        {"none", NONE},
        {"car", CAR},
        {"adult", ADULT},
        {"child", CHILD},
        {"sign", SIGN},
        {"pylon", PYLON},
        {"road", ROAD}
};


enum ObstacleStatus {
    REARCAM = -1,
    UNKNOWN = 0,
    STATIC = 1,
    DYNAMIC = 2
};

#pragma pack(push, 1)
struct tRoi {
    cv::Point2f topLeft = cv::Point2f();
    cv::Point2f bottomRight = cv::Point2f();
};
#pragma pack(pop)

#pragma pack(push, 1)
struct tClassification {
    ObstacleType type = ObstacleType::NONE;
    tFloat score = 0.f;
};
#pragma pack(pop)

static const std::size_t OBSTACLE_HULL_SIZE_MAX = 63;

#pragma pack(push, 1)
struct tObstacleData {
    tUInt64 id = 0;
    tRoi infraredROI;
    tRoi rgbROI;
    cv::Point hull[OBSTACLE_HULL_SIZE_MAX];
    std::size_t hullSize = 0;
    cv::Point2f obstacle = cv::Point2f();
    tFloat distance = 0; // in metern
    tFloat height = 0; // in metern
    tFloat radius = 0; // in metern
    tFloat certainty = 0; // sicherheit ob objekt existiert.. 0 -> unsicher 1 -> bestätigtes objekt
    ObstacleStatus status = ObstacleStatus::UNKNOWN;
    tClassification classification[5]; // top-5 classification results
};
#pragma pack(pop)

#define MEDIA_TYPE_OBSTACLE_DATA     0x00141623
#define MEDIA_SUBTYPE_OBSTACLE_DATA  0x00141623

#endif //AADC_USER_OBSTACLEDATA_H
