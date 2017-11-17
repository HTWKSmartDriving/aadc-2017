#ifndef LANE_DETECTION_FILTER_H
#define LANE_DETECTION_FILTER_H

#include "stdafx.h"
#include "ImageHelper.h"
#include <opencv2/opencv.hpp>
#include "../../HTWK_Types/StixelData.h"
#include "../../HTWK_Types/CameraProperties.h"
#include "worldOfStixel/worldOfStixel.h"

#define OID_ADTF_FILTER_DEF "htwk.stixel_world"
#define ADTF_FILTER_DESC "HTWK Stixel World"

#define STIXEL_WIDTH_DEFAULT 8
#define PROPERTY_STIXEL_WIDTH "Stixel Width"

#define PROPERTY_FREE_P1 "Free Space: Parameter 1"
#define PROPERTY_FREE_P2 "Free Space: Parameter 2"
#define PROPERTY_FREE_JUMP "Free Space: Max Pixel Jump"
#define PROPERTY_OBJECT_HEIGHT "Free Space: Object Height"
#define PROPERTY_HEIGHT_SEG_CS "Height Segmentation: Cost Smoothness"
#define PROPERTY_HEIGHT_SEG_NZ "Height Segmentation: Neighbours in Z-Direction"
#define PROPERTY_HEIGHT_SEG_DELTAZ "Height Segmentation: deltaZ"
#define PROPERTY_HEIGHT_SEG_MAX_JUMP "Height Segmentation: Max Pixel Jump"


class StixelWorldFilter : public adtf::cFilter, IThreadFunc{
ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, adtf::OBJCAT_DataFilter);

public:
    StixelWorldFilter(const tChar *__info);

    ~StixelWorldFilter();

private:
    cInputPin inputPinCameraProperty;

    cVideoPin leftInputPin;

    cVideoPin rgbInputPin;

    cVideoPin rgbOutputPin;

    cVideoPin disparityInputPin;

    cVideoPin debugStixelOutputPin;

    cVideoPin debugDisparityOutputPin;

    cOutputPin stixelOutputPin;

    tResult Init(tInitStage eStage, ucom::IException **__exception_ptr);

    tResult Stop(ucom::IException **__exception_ptr);

    tResult Shutdown(tInitStage eStage, __exception);

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

    tResult ThreadFunc(cThread *Thread, tVoid *data, tSize size);

    inline tResult transmitStixelWorld(const vector<tStixel> &stixels);

    inline tResult transmitVideo(const Mat &image, cVideoPin &pin, tBitmapFormat &bmp);

    inline void initStixelWorld(const tCameraProperties &cProp);

    unique_ptr<WorldOfStixel> stixelWorld;

    tBitmapFormat disparityInputBitmapFormat;

    tBitmapFormat debugStixelOutputBitmapFormat;

    tBitmapFormat debugDisparityOutputBitmapFormat;

    tBitmapFormat leftInputBitmapFormat;

    tBitmapFormat rgbInputBitmapFormat;

    tBitmapFormat rgbOutputBitmapFormat;

    bool disparityUpdated = false;

    bool leftImageUpdated = false;

    bool rgbImageUpdated = false;

    Mat rgbInputImage;

    Mat leftInputImage;

    Mat disparityInputImage;

    tTimeStamp disparityTimeStamp = 0;

    cThread thread;
};
#endif //LANE_DETECTION_FILTER_H
