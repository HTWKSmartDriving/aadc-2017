#ifndef WORLD_FILTER_OUT_HEADER
#define WORLD_FILTER_OUT_HEADER

#include "stdafx.h"

#include <WorldService.h>
#include <aadc_structs.h>
#include "../HTWK_Debug/EnableLogs.h"
#define OID "htwk.world_filter_out"
#define FILTER_NAME "HTWK World Out"

#define INTERVAL_PROPERTY "Interval"
#define INTERVAL_DEFAULT 30

class WorldFilterOut : public adtf::cTimeTriggeredFilter {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter)

private:
    cObjectPtr<WorldService> worldService;

    cOutputPin speedPin;
    cOutputPin steeringPin;

    cOutputPin driverStructPin;

    cOutputPin headlightsPin;
    cOutputPin breaklightsPin;
    cOutputPin turnsignalLeftPin;
    cOutputPin turnsignalRightPin;
    cOutputPin hazardlightsPin;
    cOutputPin reverselightsPin;

    cOutputPin currentSpeedPin;

    cObjectPtr<IMediaType> signalValueStruct;
    cObjectPtr<IMediaTypeDescription> signalDescription;

    cObjectPtr<IMediaType> boolSignalValueStruct;
    cObjectPtr<IMediaTypeDescription> boolSignalDescription;

    cObjectPtr<IMediaType> driverStruct;
    cObjectPtr<IMediaTypeDescription> driverStructDescription;

    tBool headlightsOnLast;
    tBool breaklightsOnLast;
    tBool turnsignalLeftOnLast;
    tBool turnsignalRightOnLast;
    tBool hazardsOnLast;
    tBool reverselightsOnLast;

public:
    WorldFilterOut(const tChar *__info);

    virtual ~WorldFilterOut();

    tResult Init(tInitStage eStage, __exception = NULL);

private:
    tResult Shutdown(tInitStage eStage, IException **__exception_ptr);

    tResult CreateDescriptions(IException **__exception_ptr);

    tResult CreateOutputPins(IException **__exception_ptr);

    tResult Cycle(IException **__exception_ptr) override;

    tResult TransmitBoolValue(cOutputPin &pin, tBool value);

    tResult TransmitSignalStruct(cOutputPin &pin, tSignalValue value);

    tResult TransmitDriverStruct(cOutputPin &pin, tInt8 state, tInt16 maneuver);
};

#endif // WORLD_FILTER_OUT_HEADER
