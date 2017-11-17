#ifndef _SIGNAL_SIGNAL_GENERATOR_HEADER_
#define _SIGNAL_SIGNAL_GENERATOR_HEADER_

#include "stdafx.h"
#include "../HTWK_Types/BehaviorTreeData.h"
#include "../HTWK_Utils/HTWKUtils.hpp"

#define OID "htwk.state_signal_generator"
#define FILTER_NAME "HTWK State Signal Generator"

#define INTERVAL_PROPERTY "Interval"
#define STATE_PROPERTY "State"

#define INTERVAL_DEFAULT 30
#define STATE_DEFAULT 0

class StateSignalGenerator : public adtf::cTimeTriggeredFilter {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter)

private:
    cOutputPin signalPin;

    cObjectPtr<IMediaType> stateSignal;
    cObjectPtr<IMediaTypeDescription> signalDescription;

public:
    explicit StateSignalGenerator(const tChar *__info);

    ~StateSignalGenerator() override;

    tResult Init(tInitStage eStage, __exception) override;

private:
    tResult Shutdown(tInitStage eStage, IException **__exception_ptr) override;

    tResult CreateDescriptions(IException **__exception_ptr);

    // tResult CreateInputPins(IException **__exception_ptr);

    tResult CreateOutputPins(IException **__exception_ptr);

    tResult Cycle(IException **__exception_ptr) override;

    // tResult TransmitValue(cOutputPin &pin, BT::tStateSignal value);

};

#endif // _SIGNAL_SIGNAL_GENERATOR_HEADER_
