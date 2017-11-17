#ifndef _SIGNAL_GENERATOR_HEADER_
#define _SIGNAL_GENERATOR_HEADER_

#include "stdafx.h"
#include "../HTWK_Debug/EnableLogs.h"
#define OID "htwk.signal_generator"
#define FILTER_NAME "HTWK Signal Generator"

#define INTERVAL_PROPERTY "Interval"
#define SIGNAL_PROPERTY "Value"

#include <chrono>

#define INTERVAL_DEFAULT 30
#define SIGNAL_DEFAULT 0

class SignalGenerator : public adtf::cTimeTriggeredFilter
{
    ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter)

    private:
        cOutputPin signalPin;

        cObjectPtr<IMediaType> signalValueStruct;
        cObjectPtr<IMediaTypeDescription> signalDescription;

public:
        SignalGenerator(const tChar *__info);

        virtual ~SignalGenerator();

        tResult Init(tInitStage eStage, __exception = NULL) override;

    private:
        tResult Shutdown(tInitStage eStage, IException **__exception_ptr) override;

        tResult CreateDescriptions(IException **__exception_ptr);

        // tResult CreateInputPins(IException **__exception_ptr);

        tResult CreateOutputPins(IException **__exception_ptr);

        tResult Cycle(IException **__exception_ptr) override;

        tResult TransmitValue(cOutputPin &pin, tFloat32 value);

};

#endif // _SIGNAL_GENERATOR_HEADER_
