#ifndef HTWK_LEAF_FILTER_H
#define HTWK_LEAF_FILTER_H

#define SPEED_MAX 2.0f

#include "stdafx.h"
#include "../../HTWK_Types/BehaviorTreeData.h"
#include <aadc_structs.h>
#include <WorldService.h>
#include <aadc_structs.h>
#include <aadc_juryEnums.h>
#include "../../HTWK_Debug/EnableLogs.h"
enum Lights {
    HEADLIGHTS = 0,
    BREAKLIGHTS = 1,
    TURNSIGNAL_LEFT = 2,
    TURNSIGNAL_RIGHT = 3,
    HAZARDLIGHTS = 4,
    REVERSELIGHTS = 5
};

class Leaf : public adtf::cFilter {

public:
    explicit Leaf(const tChar *__info);

    ~Leaf() override;

protected:
    // static pins
    IMediaSample *inputMediaSample;
    cInputPin inputPin;
    cOutputPin outputPin;
    cObjectPtr<IMediaType> mediaTypeOut;
    cObjectPtr<IMediaType> mediaTypeIn;
    cObjectPtr<WorldService> worldService;

    tResult CreateDescriptions(IException **__exception_ptr);

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

    tResult TransmitStatus(BT::moduleState status);

    tResult SetSteeringOutput(tFloat32 steeringOutput);

    tResult SetSpeedOutput(tFloat32 speedOutput);

    tResult SetCarState(stateCar state);

    tResult IncrementManeuver();

    virtual tResult OnTrigger() = 0;

    tResult SwitchLights(Lights lights, tBool on);

    void ResetLights();

    tResult Init(tInitStage eStage, __exception);


private:
    bool headlightsOn;
    bool breaklightsOn;
    bool turnsignalLeftOn;
    bool turnsignalRightOn;
    bool hazardsOn;
    bool reverselightsOn;


};

#endif // HTWK_LEAF_FILTER_H