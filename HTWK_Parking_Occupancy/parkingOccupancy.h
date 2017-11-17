//
// Created by lina on 11.11.17.
//

#ifndef AADC_USER_PARKINGOCCUPANCY_H
#define AADC_USER_PARKINGOCCUPANCY_H

#include "stdafx.h"
#include "../HTWK_Map/Orientation.h"
#include "../HTWK_Types/IntersectionState.h"
#include "aadc_juryEnums.h"
#include "../HTWK_Behavior_Tree/HTWK_LeafNode/Leaf.h"
#include "../HTWK_Types/ParkingLot.h"
#include <aadc_structs.h>
#include <iostream>
#include <HTWKPoint.hpp>
#include <HTWKUtils.hpp>
#include "../HTWK_Debug/EnableLogs.h"
#include "ImageHelper.h"
#include <opencv2/opencv.hpp>

#define OID "htwk.parkingOccupancy"
#define FILTER_NAME "HTWK BT:Node Parking Occupancy"

#define LOWER_OCCUPANCY_RATIO_PROPERTY "Lower bound occupancy ratio"
#define LOWER_OCCUPANCY_RATIO_DEFAULT 8
#define LOWER_OCCUPANCY_RATIO_DESCRIPTION "Lower bound of cells that need to be occupied to mark the parking lot as occupied"
#define LOWER_OCCUPANCY_RATIO_MIN 3
#define LOWER_OCCUPANCY_RATIO_MAX 50

class parkingOccupancy : public Leaf {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter)

public:
    explicit parkingOccupancy(const tChar *__info);

    tResult Init(tInitStage eStage, __exception) override;

    tResult Shutdown(tInitStage eStage, __exception) override;

    tResult OnTrigger() override;

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

private:
    cVideoPin gridMapInput;
    cVideoPin debugOutputPin;
    inline tResult transmitVideo(const cv::Mat &image, cVideoPin &pin, tBitmapFormat &bmp);
    tBitmapFormat debugFormat;
    vector<ParkingLot> parkingLots;
    HTWKPoint position;
    cv::Mat gridMap;
    int lowerOccRatio;

    tResult UpdateInputImageFormat(const tBitmapFormat *pFormat);

    tBitmapFormat gridMapBitmapFormat;
};


#endif //AADC_USER_PARKINGOCCUPANCY_H
