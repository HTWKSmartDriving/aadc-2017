#ifndef JURY_RESPONSE_HEADER
#define JURY_RESPONSE_HEADER

#include "stdafx.h"
#include "../HTWK_Map/Orientation.h"
#include "../HTWK_Types/IntersectionState.h"
#include "aadc_juryEnums.h"
#include "../HTWK_Behavior_Tree/HTWK_LeafNode/Leaf.h"
#include <ManeuverList.h>
#include <aadc_structs.h>
#include <iostream>
#include "../HTWK_Debug/EnableLogs.h"
#define OID "htwk.JuryResponse"
#define FILTER_NAME "HTWK BT - Node JuryResponse"

#define READY_UP_WAIT_TIME 3000000

class JuryResponse : public Leaf {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter)

private:
    tTimeStamp readyUpTimer;

    stateCar state;

public:
    JuryResponse(const tChar *__info);

    virtual ~JuryResponse();

    tResult Init(tInitStage eStage, __exception = nullptr) override;

private:
    tResult OnTrigger() override;

    tResult Shutdown(tInitStage eStage, IException **__exception_ptr) override;
};

#endif // JURY_RESPONSE_HEADER
