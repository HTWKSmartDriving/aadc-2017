#ifndef STEERING_FUSION_HEADER
#define STEERING_FUSION_HEADER

#include <WorldService.h>
#include "stdafx.h"
#include <aadc_structs.h>
#include "../HTWK_Debug/EnableLogs.h"
#define OID "htwk.steering_fusion"
#define FILTER_NAME "HTWK Steering Fusion"

class SteeringFusion : public adtf::cFilter {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter)

private:
    cInputPin laneLeftPin;
    cInputPin laneRightPin;

    tFloat32 lastLeftLane;
    tFloat32 lastRightLane;

    cObjectPtr<IMediaType> signalValueStruct;
    cObjectPtr<IMediaTypeDescription> signalDescription;

    cObjectPtr<WorldService> worldService;

    tBool m_bIDsSignalSet;

    /*! the id for the f32value of the media description for the signal value input pins */
    tBufferID m_szIDSignalF32Value;
    /*! the id for the arduino time stamp of the media description for the signal value input pins */
    tBufferID m_szIDSignalArduinoTimestamp;

public:
    SteeringFusion(const tChar *__info);

    virtual ~SteeringFusion();

    tResult Init(tInitStage eStage, __exception = nullptr) override;

private:
    tResult Shutdown(tInitStage eStage, IException **__exception_ptr) override;

    tResult CreateDescriptions(IException **__exception_ptr);

    tResult CreateInputPins(IException **__exception_ptr);

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) override;


};

#endif // STEERING_FUSION_HEADER
