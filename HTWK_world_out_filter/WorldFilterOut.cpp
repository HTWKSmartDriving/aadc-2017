#include "WorldFilterOut.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, WorldFilterOut)

WorldFilterOut::WorldFilterOut(const tChar *__info) : cTimeTriggeredFilter(__info) {
    SetPropertyInt(INTERVAL_PROPERTY, INTERVAL_DEFAULT);
    SetPropertyStr(INTERVAL_PROPERTY NSSUBPROP_DESCRIPTION, "Interval time in ms");
    SetPropertyInt(INTERVAL_PROPERTY NSSUBPROP_MIN, 10);

     headlightsOnLast = tFalse;
     breaklightsOnLast = tFalse;
     turnsignalLeftOnLast = tFalse;
     turnsignalRightOnLast = tFalse;
     hazardsOnLast = tFalse;
     reverselightsOnLast = tFalse;
}

WorldFilterOut::~WorldFilterOut() {
}

tResult WorldFilterOut::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cTimeTriggeredFilter::Init(eStage, __exception_ptr));

    if (StageFirst == eStage) {
        RETURN_IF_FAILED(CreateDescriptions(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));

    } else if (StageNormal == eStage) {
        auto interval = tUInt(GetPropertyInt(INTERVAL_PROPERTY));
        SetInterval(interval * 1000);

    } else if (StageGraphReady == eStage) {
        RETURN_IF_FAILED(
                _runtime->GetObject(OID_WORLD_SERVICE, IID_WORLD_INTERFACE, (tVoid **) &worldService, __exception_ptr));
    }

    RETURN_NOERROR;
}

tResult WorldFilterOut::Shutdown(tInitStage eStage, __exception) {
    return cTimeTriggeredFilter::Shutdown(eStage, __exception_ptr);
}

tResult WorldFilterOut::CreateDescriptions(IException **__exception_ptr) {
    cObjectPtr<IMediaDescriptionManager> descManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                         (tVoid **) &descManager, __exception_ptr));

    //get SignalValue
    tChar const *signalValueDescription = descManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(signalValueDescription);
    signalValueStruct = new cMediaType(0, 0, 0, "tSignalValue", signalValueDescription,
                                       IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(signalValueStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &signalDescription));

    tChar const *boolSignalValueDescription = descManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(boolSignalValueDescription);
    boolSignalValueStruct = new cMediaType(0, 0, 0, "tBoolSignalValue", boolSignalValueDescription,
                                           IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(
            boolSignalValueStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &boolSignalDescription));

    tChar const *driverDescription = descManager->GetMediaDescription("tDriverStruct");
    RETURN_IF_POINTER_NULL(driverDescription);
    driverStruct = new cMediaType(0, 0, 0, "tDriverStruct", driverDescription,
                                  IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(
            driverStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &driverStructDescription));

    RETURN_NOERROR;
}

tResult WorldFilterOut::CreateOutputPins(IException **__exception_ptr) {
    RETURN_IF_FAILED(driverStructPin.Create("driver_struct", driverStruct, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&driverStructPin));

    RETURN_IF_FAILED(steeringPin.Create("steeringController", signalValueStruct, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&steeringPin));

    RETURN_IF_FAILED(speedPin.Create("speedController", signalValueStruct, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&speedPin));

    RETURN_IF_FAILED(currentSpeedPin.Create("current_speed", signalValueStruct, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&currentSpeedPin));

    RETURN_IF_FAILED(headlightsPin.Create("headlights", boolSignalValueStruct, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&headlightsPin));

    RETURN_IF_FAILED(breaklightsPin.Create("breaklights", boolSignalValueStruct, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&breaklightsPin));

    RETURN_IF_FAILED(
            turnsignalLeftPin.Create("turnsignal_left", boolSignalValueStruct, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&turnsignalLeftPin));

    RETURN_IF_FAILED(
            turnsignalRightPin.Create("turnsignal_right", boolSignalValueStruct, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&turnsignalRightPin));

    RETURN_IF_FAILED(
            hazardlightsPin.Create("hazardlights", boolSignalValueStruct, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&hazardlightsPin));

    RETURN_IF_FAILED(
            reverselightsPin.Create("reverselights", boolSignalValueStruct, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&reverselightsPin));

    RETURN_NOERROR;
}

tResult WorldFilterOut::Cycle(IException **__exception_ptr) {
    tSignalValue steeringStruct{};
    tSignalValue speedStruct{};

    if (IS_OK(worldService->Pull(CAR_STEERING, steeringStruct))) {
        TransmitSignalStruct(steeringPin, steeringStruct);
    } else {
#ifdef DEBUG_WORLD_FILTER_OUT_LOG
        LOG_ERROR("Steering struct could not be pulled from WorldService");
#endif
    }

    if (IS_OK(worldService->Pull(CAR_SPEED, speedStruct))) {
        TransmitSignalStruct(speedPin, speedStruct);
    } else {
#ifdef DEBUG_WORLD_FILTER_OUT_LOG
        LOG_ERROR("Speed struct could not be pulled from WorldService");
#endif
    }

    tInt16 maneuver;
    tInt8 state;

    if (IS_OK(worldService->Pull(CAR_MANEUVER_ENTRY, maneuver))) {
        if (IS_OK(worldService->Pull(CAR_STATE, state))) {
            TransmitDriverStruct(driverStructPin, state, maneuver);
        } else {
#ifdef DEBUG_WORLD_FILTER_OUT_LOG
            LOG_ERROR("Car state could not be pulled from WorldService");
#endif
        }
    } else {
#ifdef DEBUG_WORLD_FILTER_OUT_LOG
        LOG_ERROR("Maneuver could not be pulled from WorldService");
#endif
    }

    tBool headlightsOn;
    tBool breaklightsOn;
    tBool turnsignalLeftOn;
    tBool turnsignalRightOn;
    tBool hazardsOn;
    tBool reverselightsOn;

    if (IS_OK(worldService->Pull(CAR_HEADLIGHTS, headlightsOn))) {
        if(headlightsOn != headlightsOnLast)
        {
            TransmitBoolValue(headlightsPin, headlightsOn);
            headlightsOnLast = headlightsOn;
        }
    } else {
#ifdef DEBUG_WORLD_FILTER_OUT_LOG
        LOG_ERROR("Headlights could not be pulled from WorldService");
#endif
    }

    if (IS_OK(worldService->Pull(CAR_BREAKLIGHTS, breaklightsOn))) {
        if(breaklightsOn != breaklightsOnLast)
        {
            TransmitBoolValue(breaklightsPin, breaklightsOn);
            breaklightsOnLast = breaklightsOn;
        }
    } else {
#ifdef DEBUG_WORLD_FILTER_OUT_LOG
        LOG_ERROR("Breaklights could not be pulled from WorldService");
#endif
    }

    if (IS_OK(worldService->Pull(CAR_TURNSIGNAL_LEFT, turnsignalLeftOn))) {
        if(turnsignalLeftOn != turnsignalLeftOnLast)
        {
            TransmitBoolValue(turnsignalLeftPin, turnsignalLeftOn);
            turnsignalLeftOnLast = turnsignalLeftOn;
        }
    } else {
#ifdef DEBUG_WORLD_FILTER_OUT_LOG
        LOG_ERROR("turnsignalLeft could not be pulled from WorldService");
#endif
    }

    if (IS_OK(worldService->Pull(CAR_TURNSIGNAL_RIGHT, turnsignalRightOn))) {
        if (turnsignalRightOn != turnsignalRightOnLast) {
            TransmitBoolValue(turnsignalRightPin, turnsignalRightOn);
            turnsignalRightOnLast = turnsignalRightOn;
        }
    }else {
#ifdef DEBUG_WORLD_FILTER_OUT_LOG
        LOG_ERROR("turnsignalRight could not be pulled from WorldService");
#endif
    }

    if (IS_OK(worldService->Pull(CAR_HAZARDLIGHTS, hazardsOn))) {
        if (hazardsOn != hazardsOnLast) {
            TransmitBoolValue(hazardlightsPin, hazardsOn);
            hazardsOnLast = hazardsOn;
        }
    } else {
#ifdef DEBUG_WORLD_FILTER_OUT_LOG
        LOG_ERROR("hazards could not be pulled from WorldService");
#endif
    }

    if (IS_OK(worldService->Pull(CAR_REVERSELIGHTS, reverselightsOn))) {
        if (reverselightsOn != reverselightsOnLast) {
            TransmitBoolValue(reverselightsPin, reverselightsOn);
            reverselightsOnLast = reverselightsOn;
        }
    } else {
#ifdef DEBUG_WORLD_FILTER_OUT_LOG
        LOG_ERROR("reverselights could not be pulled from WorldService");
#endif
    }

    tFloat32 currentSpeed;
    if (IS_OK(worldService->Pull(WORLD_CURRENT_SPEED, currentSpeed))) {
        tSignalValue currentSpeedStruct{0, currentSpeed};
#ifdef DEBUG_WORLD_FILTER_OUT_LOG
        LOG_INFO(adtf_util::cString::Format("SpeedOut: %f", currentSpeed));
#endif
        TransmitSignalStruct(currentSpeedPin, currentSpeedStruct);
    } else {
#ifdef DEBUG_WORLD_FILTER_OUT_LOG
        LOG_ERROR("Current speed could not be pulled from WorldService");
#endif
    }

    RETURN_NOERROR;
}

tResult WorldFilterOut::TransmitBoolValue(cOutputPin &pin, tBool value) {
/*    if (!pin.IsConnected())
    {
        RETURN_NOERROR;
    }*/

    cObjectPtr<IMediaSample> mediaSample;
    AllocMediaSample((tVoid **) &mediaSample);

    cObjectPtr<IMediaSerializer> serializer;
    boolSignalDescription->GetMediaSampleSerializer(&serializer);
    mediaSample->AllocBuffer(serializer->GetDeserializedSize());

    tBool boolValue = value;
    tUInt32 ui32TimeStamp = 0;

    {
        __adtf_sample_write_lock_mediadescription(boolSignalDescription, mediaSample, pCoderOutput);
        pCoderOutput->Set("bValue", (tVoid *) &boolValue);
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid *) &(ui32TimeStamp));
    }

    mediaSample->SetTime(_clock->GetStreamTime());
    pin.Transmit(mediaSample);

    RETURN_NOERROR;
}


tResult WorldFilterOut::TransmitSignalStruct(cOutputPin &pin, tSignalValue value) {
/*    if (!pin.IsConnected())
    {
        RETURN_NOERROR;
    }*/

    cObjectPtr<IMediaSample> mediaSample;
    AllocMediaSample((tVoid **) &mediaSample);

    cObjectPtr<IMediaSerializer> serializer;
    signalDescription->GetMediaSampleSerializer(&serializer);
    mediaSample->AllocBuffer(serializer->GetDeserializedSize());

    tFloat32 f32Value = value.f32Value;
    tUInt32 ui32TimeStamp = value.ui32ArduinoTimestamp;

    {
        __adtf_sample_write_lock_mediadescription(signalDescription, mediaSample, pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid *) &f32Value);
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid *) &(ui32TimeStamp));
    }

    mediaSample->SetTime(_clock->GetStreamTime());
    pin.Transmit(mediaSample);

    RETURN_NOERROR;
}

tResult WorldFilterOut::TransmitDriverStruct(cOutputPin &pin, tInt8 state, tInt16 maneuver) {
/*    if (!pin.IsConnected())
    {
        RETURN_NOERROR;
    }*/

    cObjectPtr<IMediaSample> mediaSample;
    AllocMediaSample((tVoid **) &mediaSample);

    cObjectPtr<IMediaSerializer> serializer;
    driverStructDescription->GetMediaSampleSerializer(&serializer);
    mediaSample->AllocBuffer(serializer->GetDeserializedSize());

    tInt8 stateId = state;
    tInt16 maneuverEntry = maneuver;

    {
        __adtf_sample_write_lock_mediadescription(driverStructDescription, mediaSample, pCoderOutput);
        pCoderOutput->Set("i8StateID", (tVoid *) &stateId);
        pCoderOutput->Set("i16ManeuverEntry", (tVoid *) &maneuverEntry);
    }

    mediaSample->SetTime(_clock->GetStreamTime());
    pin.Transmit(mediaSample);

    RETURN_NOERROR;
}