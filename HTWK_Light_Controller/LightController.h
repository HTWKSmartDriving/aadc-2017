#ifndef LIGHT_CONTROLLER_HEADER
#define LIGHT_CONTROLLER_HEADER

#include "stdafx.h"
#include "../HTWK_Types/IntersectionState.h"
#include "../HTWK_Map/Orientation.h"
#include "../HTWK_Behavior_Tree/HTWK_LeafNode/Leaf.h"
#include <aadc_juryEnums.h>
#include "../HTWK_Debug/EnableLogs.h"
#define OID "htwk.LightController"
#define FILTER_NAME "HTWK BT - Node LightController"

#define BREAK_TIME_OFFSET_PROPERTY "Break sampling rate"
#define BREAK_TIME_OFFSET 0.5f

#define TO_MICRO_SEC 1000000
#define BREAK_ACCELERATION 0.1f

class LightController : public Leaf {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter)

private:
    tFloat32 lastSpeed;
    tTimeStamp breakLightTimer;
    tTimeStamp breakTimeOffset;

public:
    LightController(const tChar *__info);

    virtual ~LightController();

    tResult Init(tInitStage eStage, __exception = nullptr) override;

private:
    tResult OnTrigger() override;

    tResult Shutdown(tInitStage eStage, IException **__exception_ptr) override;
};

#endif // LIGHT_CONTROLLER_HEADER
