#include "Leaf.h"

Leaf::Leaf(const tChar *__info) : cFilter(__info) {}

Leaf::~Leaf() = default;

tResult Leaf::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {
        //this is a static Pin, creates Media description and output pin
        RETURN_IF_FAILED(CreateDescriptions(__exception_ptr));

        // create all static pins
        RETURN_IF_FAILED(inputPin.Create("input", mediaTypeIn, this));
        RETURN_IF_FAILED(RegisterPin(&inputPin));

        RETURN_IF_FAILED(outputPin.Create("output", mediaTypeOut, this));
        RETURN_IF_FAILED(RegisterPin(&outputPin));
    } else if (StageGraphReady == eStage) {
        RETURN_IF_FAILED(
                _runtime->GetObject(OID_WORLD_SERVICE, IID_WORLD_INTERFACE, (tVoid **) &worldService, __exception_ptr));
    }

    RETURN_NOERROR;
}

tResult Leaf::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) {

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        RETURN_IF_POINTER_NULL(pMediaSample);
        if (pSource == &inputPin) {
            inputMediaSample = pMediaSample;
            bool receivedTrigger;
            {
                // this will aquire the read lock on the sample and declare and initialize a pointer to the data
                __sample_read_lock(pMediaSample, tBoolSignalValue, pData);
                // now we can access the sample data through the pointer
                receivedTrigger = pData->bValue;
            }
            if (receivedTrigger) {
                OnTrigger();
            }
        }
    }

    RETURN_NOERROR;
}

tResult Leaf::CreateDescriptions(IException **__exception_ptr) {
    cObjectPtr<IMediaDescriptionManager> descManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                         (tVoid **) &descManager, __exception_ptr));

    tChar const *resultOutputDescription = descManager->GetMediaDescription("tStateSignal");
    RETURN_IF_POINTER_NULL(resultOutputDescription);

    mediaTypeOut = new cMediaType(0, 0, 0, "tStateSignal", resultOutputDescription, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(mediaTypeOut->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &resultOutputDescription));

    resultOutputDescription = descManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(resultOutputDescription);

    mediaTypeIn = new cMediaType(0, 0, 0, "tBoolSignalValue", resultOutputDescription, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(mediaTypeIn->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &resultOutputDescription));

    RETURN_NOERROR;
}

tResult Leaf::TransmitStatus(BT::moduleState status) {
    BT::tStateSignal transmittingValue{status, 0};
    transmittingValue.state = status;
    cObjectPtr<IMediaSample> resultData;
    if (IS_OK(AllocMediaSample(&resultData))) {
        resultData->Update(inputMediaSample->GetTime(), &transmittingValue, sizeof(BT::tStateSignal), 0);
        outputPin.Transmit(resultData);
    }

    RETURN_NOERROR;
}

tResult Leaf::SetSteeringOutput(tFloat32 steeringOutput) {
    tSignalValue steeringStruct{static_cast<tUInt32>(_clock->GetStreamTime()),
                                steeringOutput * (-100.0f / 30.0f)};
    worldService->Push(CAR_STEERING, steeringStruct);
    RETURN_NOERROR;
}

tResult Leaf::SetSpeedOutput(tFloat32 speedOutput) {
    if(speedOutput < SPEED_MAX) {
        tSignalValue speedStruct{static_cast<tUInt32>(_clock->GetStreamTime()), speedOutput};
        worldService->Push(CAR_SPEED, speedStruct);
    } else {
        tSignalValue speedStruct{static_cast<tUInt32>(_clock->GetStreamTime()), SPEED_MAX};
        worldService->Push(CAR_SPEED, speedStruct);
    }

    RETURN_NOERROR;
}

tResult Leaf::SetCarState(stateCar state) {
    worldService->Push(CAR_STATE, state);
    RETURN_NOERROR;
}

tResult Leaf::IncrementManeuver() {
    tInt32 currentManeuverId;
    worldService->Pull(CAR_MANEUVER_ENTRY, currentManeuverId);
    worldService->Push(CAR_MANEUVER_ENTRY, ++currentManeuverId);
    RETURN_NOERROR;
}

tResult Leaf::SwitchLights(Lights lights, tBool on) {
    switch (lights) {
        case HEADLIGHTS:
            if (headlightsOn != on) {
                worldService->Push(CAR_HEADLIGHTS, on);
                headlightsOn = on;
            }
            break;
        case BREAKLIGHTS:
            if (breaklightsOn != on) {
                worldService->Push(CAR_BREAKLIGHTS, on);
                breaklightsOn = on;
            }
            break;
        case TURNSIGNAL_LEFT:
            // turn off right blinker first
            if (turnsignalRightOn) {
                worldService->Push(CAR_TURNSIGNAL_RIGHT, false);
                turnsignalRightOn = false;
            }

            if (turnsignalLeftOn != on) {
                worldService->Push(CAR_TURNSIGNAL_LEFT, on);
                turnsignalLeftOn = on;
            }
            break;
        case TURNSIGNAL_RIGHT:
            // turn off left blinker first
            if (turnsignalLeftOn) {
                worldService->Push(CAR_TURNSIGNAL_LEFT, false);
                turnsignalLeftOn = false;
            }

            if (turnsignalRightOn != on) {
                worldService->Push(CAR_TURNSIGNAL_RIGHT, on);
                turnsignalRightOn = on;
            }
            break;
        case HAZARDLIGHTS:
            if (hazardsOn != on) {
                worldService->Push(CAR_HAZARDLIGHTS, on);
                hazardsOn = on;
            }
            break;
        case REVERSELIGHTS:
            if (reverselightsOn != on) {
                worldService->Push(CAR_REVERSELIGHTS, on);
                reverselightsOn = on;
            }
            break;
    }

    RETURN_NOERROR;
}

void Leaf::ResetLights() {
    headlightsOn = tFalse;
    breaklightsOn = tFalse;
    turnsignalLeftOn = tFalse;
    turnsignalRightOn = tFalse;
    hazardsOn = tFalse;
    reverselightsOn = tFalse;

    worldService->Push(CAR_HEADLIGHTS, tFalse);
    worldService->Push(CAR_BREAKLIGHTS, tFalse);
    worldService->Push(CAR_TURNSIGNAL_LEFT, tFalse);
    worldService->Push(CAR_TURNSIGNAL_RIGHT, tFalse);
    worldService->Push(CAR_HAZARDLIGHTS, tFalse);
    worldService->Push(CAR_REVERSELIGHTS, tFalse);
}

