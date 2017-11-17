#ifndef HTWK_US_OBSTACLE_DETECTION
#define HTWK_US_OBSTACLE_DETECTION

#include "stdafx.h"
#include "../HTWK_Behavior_Tree/HTWK_LeafNode/Leaf.h"
#include "../HTWK_Types/BehaviorTreeData.h"
#include "../HTWK_Debug/EnableLogs.h"
#include "../HTWK_Types/IntersectionState.h"

#define OID "htwk.brake"
#define FILTER_NAME "HTWK Brake Filter"
#define standTime 4000000;


#define OVERTAKE_PROPERTY "OVERTAKE_"
#define OVERTAKE_DEFAULT true

class Brake : public Leaf {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_Auxiliary);

private:
    bool isOvertake;
    bool turnOut = false;
    tTimeStamp stopTime;

    IntersectionState interState;

    tResult OnTrigger();


public:
    Brake(const tChar *__info);

    virtual ~Brake();

    tResult Init(tInitStage eStage, __exception) override;
};

#endif //HTWK_US_OBSTACLE_DETECTION