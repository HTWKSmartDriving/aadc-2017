#include "LaneWorldOut.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, LaneWorldOut)

LaneWorldOut::LaneWorldOut(const tChar *__info) : Leaf(__info) {
    SetPropertyFloat(ROAD_SPEED_PROPERTY, LANE_DRIVING_SPEED);
    SetPropertyStr(ROAD_SPEED_PROPERTY NSSUBPROP_DESCRIPTION, "Speed to drive on road");
}

LaneWorldOut::~LaneWorldOut()
= default;

tResult LaneWorldOut::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(Leaf::Init(eStage, __exception_ptr));

    if (StageFirst == eStage) {
        RETURN_IF_FAILED(CreateDescriptionsLanes(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
    } else if (StageNormal == eStage) {

    } else if (StageGraphReady == eStage) {
        speedOnRoad = static_cast<tFloat32>(GetPropertyFloat(ROAD_SPEED_PROPERTY));
    }

    RETURN_NOERROR;
}

tResult LaneWorldOut::Shutdown(tInitStage eStage, __exception) {
    return Leaf::Shutdown(eStage, __exception_ptr);
}


tResult LaneWorldOut::CreateDescriptionsLanes(IException **__exception_ptr) {
    cObjectPtr<IMediaDescriptionManager> descManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                         (tVoid **) &descManager, __exception_ptr));

    //get SignalValue
    tChar const *signalValueDescription = descManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(signalValueDescription);
    signalValueStruct = new cMediaType(0, 0, 0, "tSignalValue", signalValueDescription,
                                       IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(signalValueStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &signalDescription));

    RETURN_NOERROR;
}

tResult LaneWorldOut::CreateOutputPins(IException **__exception_ptr) {
    RETURN_IF_FAILED(leftLaneOutput.Create("left_lane", signalValueStruct, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&leftLaneOutput));

    RETURN_IF_FAILED(rightLaneOutput.Create("right_lane", signalValueStruct, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&rightLaneOutput));

    RETURN_IF_FAILED(centerOutput.Create("center", signalValueStruct, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&centerOutput));

    RETURN_NOERROR;
}

tResult LaneWorldOut::OnTrigger() {

    tFloat32 leftLane;
    if (IS_OK(worldService->Pull(WORLD_LANE_LEFT, leftLane))) {
        TransmitValue(leftLaneOutput, leftLane);
    } else {
        TransmitStatus(BT::FAIL);
#ifdef DEBUG_LANE_WORLD_OUT_LOG
        LOG_ERROR("Left lane could not be pulled from WorldService");
#endif
    }

    tFloat32 rightLane;
    if (IS_OK(worldService->Pull(WORLD_LANE_RIGHT, rightLane))) {
        TransmitValue(rightLaneOutput, rightLane);
    } else {
        TransmitStatus(BT::FAIL);
#ifdef DEBUG_LANE_WORLD_OUT_LOG
        LOG_ERROR("Right lane could not be pulled from WorldService");
#endif
    }

    tFloat32 center;
    if (IS_OK(worldService->Pull(WORLD_LANE_CENTER, center))) {
        TransmitValue(centerOutput, center);
    } else {
        TransmitStatus(BT::FAIL);
#ifdef DEBUG_LANE_WORLD_OUT_LOG
        LOG_ERROR("Center could not be pulled from WorldService");
#endif
    }

    stateCar state;
    if (!IS_OK(worldService->Pull(CAR_STATE, state))) {
        TransmitStatus(BT::FAIL);
#ifdef DEBUG_LANE_WORLD_OUT_LOG
        LOG_ERROR("Car state could not be pulled from WorldService");
#endif
    }

    findSpeedAndPush(state);

    TransmitStatus(BT::RUNNING);
    RETURN_NOERROR;
}

tResult LaneWorldOut::TransmitValue(cOutputPin &pin, tFloat32 value) {
    if (!pin.IsConnected()) {
        RETURN_NOERROR;
    }

    cObjectPtr<IMediaSample> mediaSample;
    AllocMediaSample((tVoid **) &mediaSample);

    cObjectPtr<IMediaSerializer> serializer;
    signalDescription->GetMediaSampleSerializer(&serializer);
    mediaSample->AllocBuffer(serializer->GetDeserializedSize());

    tFloat32 f32Value = value;
    tUInt32 ui32TimeStamp = 0;

    {
        __adtf_sample_write_lock_mediadescription(signalDescription, mediaSample, pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid *) &f32Value);
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid *) &(ui32TimeStamp));
    }

    mediaSample->SetTime(_clock->GetStreamTime());
    pin.Transmit(mediaSample);

    RETURN_NOERROR;
}

void LaneWorldOut::findSpeedAndPush(stateCar state) {
    tFloat32 speed = 0;
    if(state == stateCar::stateCar_RUNNING) {
        speed = speedOnRoad;
    } else {
        speed = 0;
    }

    SetSpeedOutput(speed);

#ifdef DEBUG_LANE_WORLD_OUT_LOG
    LOG_INFO("LaneDriving active!");
#endif
}
