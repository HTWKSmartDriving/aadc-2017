#ifndef PROJECT_TYPES_H
#define PROJECT_TYPES_H

#include <opencv2/opencv.hpp>
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

#pragma pack(push, 1)
struct tIntrinsics {
    cv::Point2f focal = cv::Point2f();
    cv::Point2f principal = cv::Point2f();
};
#pragma pack(pop)

#pragma pack(push, 1)
struct tExtrensics {
    float rotation[9] = {0};
    float translation[3] = {0};
};
#pragma pack(pop)

#pragma pack(push, 1)
struct tCameraProperties {
    const tFloat base = 0.07; //See realsense documentation
    const tFloat height = 0.24;
    tInt32 disparity = 64;
    tExtrensics exIRtoRGB;
    tIntrinsics inInfraredLeft;
    tIntrinsics inRGB;
};
#pragma pack(pop)

#define MEDIA_TYPE_CAMERA_PROPERTY    0x00040608
#define MEDIA_SUBTYPE_CAMERA_PROPERTY  0x00040608

#endif //PROJECT_TYPES_H
