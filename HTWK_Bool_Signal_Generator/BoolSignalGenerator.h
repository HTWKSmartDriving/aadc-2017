#ifndef _SIGNAL_GENERATOR_HEADER_
#define _SIGNAL_GENERATOR_HEADER_

#include "stdafx.h"
#include <aadc_structs.h>
#include "../HTWK_Debug/EnableLogs.h"
#define OID "htwk.bool_signal_generator"
#define FILTER_NAME "HTWK Bool Signal Generator"

#define INTERVAL_PROPERTY "Interval"
#define SIGNAL_PROPERTY "Value"

#define INTERVAL_DEFAULT 30
#define SIGNAL_DEFAULT false

class BoolSignalGenerator : public adtf::cTimeTriggeredFilter
{
    ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter)

    private:
        cOutputPin signalPin;

        cObjectPtr<IMediaType> signalValueStruct;
        cObjectPtr<IMediaTypeDescription> signalDescription;

public:
        BoolSignalGenerator(const tChar *__info);

        virtual ~BoolSignalGenerator();

        tResult Init(tInitStage eStage, __exception = NULL) override;

    private:
        tResult Shutdown(tInitStage eStage, IException **__exception_ptr) override;

        tResult CreateDescriptions(IException **__exception_ptr);

        // tResult CreateInputPins(IException **__exception_ptr);

        tResult CreateOutputPins(IException **__exception_ptr);

        tResult Cycle(IException **__exception_ptr) override;

        tResult TransmitValue(cOutputPin &pin, tBool value);

};

#endif // _SIGNAL_GENERATOR_HEADER_
