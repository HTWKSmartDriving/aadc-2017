#ifndef _LANE_Steering_FILTER_H_
#define _LANE_Steering_FILTER_H_

#include "stdafx.h"
#include "../../HTWK_Types/LaneData.h"
#include "../../HTWK_Types/CameraResolutionData.h"
#include "LaneSteeringCalculator.h"
#include "../../HTWK_Debug/EnableLogs.h"
#define OID_ADTF_FILTER_DEF "htwk.lane_steering"
#define ADTF_FILTER_DESC "HTWK Lane Steering"

class LaneSteeringFilter : public adtf::cFilter {
ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, adtf::OBJCAT_DataFilter);

public:
    LaneSteeringFilter(const tChar *__info);

protected:

    cInputPin inputPinTypeLanePoint;

    cInputPin inputPinTypeCameraResolutionData;

    cOutputPin outputPinLeftLane;
    cOutputPin outputPinRightLane;

    cOutputPin outputPinCenter;

    cObjectPtr<IMediaType> signalValueStruct;
    cObjectPtr<IMediaTypeDescription> signalDescription;


    tResult Init(tInitStage eStage, ucom::IException **__exception_ptr);

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

    tResult TransmitValue(cOutputPin &pin, const double &value);

private:
    std::unique_ptr<LaneSteeringCalculator> laneSteeringCalculator;

    void calcSteering(const IMediaSample *pMediaSample, const vector<tLanePoint> &tLanePointVector);
};

#endif // _LANE_Steering_FILTER_H_
