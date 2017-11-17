#ifndef LANE_WORLD_OUT_HEADER
#define LANE_WORLD_OUT_HEADER

#include "stdafx.h"
#include "../HTWK_Behavior_Tree/HTWK_LeafNode/Leaf.h"
#include <aadc_juryEnums.h>
#include "../HTWK_Debug/EnableLogs.h"
#define OID "htwk.lane_world_out"
#define FILTER_NAME "HTWK BT - Node Lane World Out"

#define ROAD_SPEED_PROPERTY "roadSpeed"
#define LANE_DRIVING_SPEED 0.8f

class LaneWorldOut : public Leaf {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter)

private:
    cOutputPin leftLaneOutput;
    cOutputPin rightLaneOutput;
    cOutputPin centerOutput;

    cObjectPtr<IMediaType> signalValueStruct;
    cObjectPtr<IMediaTypeDescription> signalDescription;

    tFloat32 speedOnRoad;

public:
    LaneWorldOut(const tChar *__info);

    virtual ~LaneWorldOut();

    tResult Init(tInitStage eStage, __exception = NULL) override;

private:
    tResult OnTrigger() override;

    tResult Shutdown(tInitStage eStage, IException **__exception_ptr) override;

    tResult CreateDescriptionsLanes(IException **__exception_ptr);

    tResult CreateOutputPins(IException **__exception_ptr);

    tResult TransmitValue(cOutputPin &pin, tFloat32 value);

    void findSpeedAndPush(stateCar state);
};

#endif // LANE_WORLD_OUT_HEADER
