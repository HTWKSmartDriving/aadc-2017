#include "StateSignalGenerator.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, StateSignalGenerator)

StateSignalGenerator::StateSignalGenerator(const tChar *__info) : cTimeTriggeredFilter(__info) {
    SetPropertyInt(INTERVAL_PROPERTY, INTERVAL_DEFAULT);
    SetPropertyStr(INTERVAL_PROPERTY NSSUBPROP_DESCRIPTION, "Interval time in ms");
    SetPropertyInt(INTERVAL_PROPERTY NSSUBPROP_MIN, 10);

    SetPropertyInt(STATE_PROPERTY, STATE_DEFAULT);
    SetPropertyStr(STATE_PROPERTY NSSUBPROP_VALUELIST, "0@FAIL|1@SUCCESS|2@RUNNING");
    SetPropertyStr(STATE_PROPERTY NSSUBPROP_DESCRIPTION, "State to transmit (FAIL, SUCCESS or RUNNING)");
}

StateSignalGenerator::~StateSignalGenerator() = default;

tResult StateSignalGenerator::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (StageFirst == eStage) {
        RETURN_IF_FAILED(CreateDescriptions(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
    } else if (StageNormal == eStage) {
        auto interval = tUInt(GetPropertyInt(INTERVAL_PROPERTY));
        SetInterval(interval * 1000);
    }

    RETURN_NOERROR;
}

tResult StateSignalGenerator::Shutdown(tInitStage eStage, __exception) {
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult StateSignalGenerator::CreateDescriptions(IException **__exception_ptr) {
    cObjectPtr<IMediaDescriptionManager> descManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                         (tVoid **) &descManager, __exception_ptr));

    //get SignalValue
    tChar const *signalValueDescription = descManager->GetMediaDescription("tStateSignal");
    RETURN_IF_POINTER_NULL(signalValueDescription);
    stateSignal = new cMediaType(0, 0, 0, "tStateSignal", signalValueDescription, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(stateSignal->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &signalDescription));

    RETURN_NOERROR;
}

tResult StateSignalGenerator::CreateOutputPins(IException **__exception_ptr) {
    RETURN_IF_FAILED(signalPin.Create("signal", stateSignal, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&signalPin));

    RETURN_NOERROR;
}

tResult StateSignalGenerator::Cycle(IException **__exception_ptr) {
    BT::tStateSignal stateSignal = {static_cast<BT::moduleState>(GetPropertyInt(STATE_PROPERTY)), 0};
    cObjectPtr<IMediaSample> resultData;

    if (IS_OK(AllocMediaSample(&resultData))) {
        resultData->Update(_clock->GetStreamTime(), &stateSignal, sizeof(BT::tStateSignal), 0);
        signalPin.Transmit(resultData);
    }

    RETURN_NOERROR;
}