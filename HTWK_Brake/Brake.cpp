#include "Brake.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, Brake);

Brake::Brake(const tChar *__info) : Leaf(__info) {

    SetPropertyBool(OVERTAKE_PROPERTY, OVERTAKE_DEFAULT);
}

Brake::~Brake()
= default;

tResult Brake::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(Leaf::Init(eStage, __exception_ptr));
    if (eStage == StageNormal) {
        isOvertake = GetPropertyBool(OVERTAKE_PROPERTY, OVERTAKE_DEFAULT);
    }
    RETURN_NOERROR;
}

tResult Brake::OnTrigger() {
    if (!IS_OK(worldService->Pull(WORLD_INTERSECTION_STATE, interState))) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_ERROR("Inter state could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if(interState == IntersectionState::ON_ROAD) {
        if (isOvertake) {
            if (!IS_OK(worldService->Pull(CAR_AVOID_MOVE, turnOut))) {
                worldService->Push(CAR_AVOID_MOVE, false);
#ifdef DEBUG_BRAKE_LOG
                LOG_INFO("init");
#endif
            }
        }


        if (!turnOut) {
#ifdef DEBUG_BRAKE_LOG
            LOG_INFO("stop");
#endif
            stopTime = _clock->GetStreamTime() + standTime;
            turnOut = true;
            if (isOvertake) {
                worldService->Push(CAR_AVOID_MOVE, turnOut);
            }
            SetSpeedOutput(0);
            TransmitStatus(BT::RUNNING);
            RETURN_NOERROR;
        } else {
            if (_clock->GetStreamTime() < stopTime) {
                SetSpeedOutput(0);
#ifdef DEBUG_BRAKE_LOG
                LOG_INFO("wait for obstacles");
#endif
                TransmitStatus(BT::RUNNING);
                RETURN_NOERROR;
            } else {
                //turnOut
#ifdef DEBUG_BRAKE_LOG
                LOG_INFO("turn out");
#endif
//            worldService->Push(CAR_AVOID_MOVE, &turnOut);
                if (!isOvertake) {
                    turnOut = false;
                }
                TransmitStatus(BT::FAIL);
                RETURN_NOERROR;
            }
        }
    } else {
        SetSpeedOutput(0);
        TransmitStatus(BT::RUNNING);
        RETURN_NOERROR;
    }
}
