#include "LightController.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, LightController)

LightController::LightController(const tChar *__info) : Leaf(__info) {
    SetPropertyFloat(BREAK_TIME_OFFSET_PROPERTY, BREAK_TIME_OFFSET);
    SetPropertyStr(BREAK_TIME_OFFSET_PROPERTY NSSUBPROP_DESCRIPTION, "Sampling time to detect breaking");

    breakLightTimer = 0;
}

LightController::~LightController()
= default;

tResult LightController::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(Leaf::Init(eStage, __exception_ptr));

    if (StageFirst == eStage) {

    } else if (StageNormal == eStage) {
        breakTimeOffset = static_cast<tTimeStamp>(GetPropertyFloat(BREAK_TIME_OFFSET_PROPERTY) * 1000000);

    } else if (StageGraphReady == eStage) {
        ResetLights();
    }

    RETURN_NOERROR;
}

tResult LightController::Shutdown(tInitStage eStage, __exception) {
    return Leaf::Shutdown(eStage, __exception_ptr);
}

tResult LightController::OnTrigger() {
    tFloat32 currentSpeed;
    stateCar state;

    if(breakLightTimer == 0) {
        breakLightTimer = _clock->GetStreamTime();
        lastSpeed = 0;
    }

    if (!IS_OK(worldService->Pull(CAR_STATE, state))) {
#ifdef DEBUG_LIGHT_CONTROLLER_LOG
        LOG_ERROR("Car state state could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (state == stateCar::stateCar_RUNNING) {
        SwitchLights(HEADLIGHTS, tTrue);

        if (!IS_OK(worldService->Pull(WORLD_CURRENT_SPEED, currentSpeed))) {
#ifdef DEBUG_LIGHT_CONTROLLER_LOG
            LOG_ERROR("Current speed could not be pulled from WorldService");
#endif
        } else {
            if (std::abs(currentSpeed) < (std::abs(lastSpeed) - BREAK_ACCELERATION)) {
                SwitchLights(BREAKLIGHTS, tTrue);
            } else {
                SwitchLights(BREAKLIGHTS, tFalse);
            }

            if (currentSpeed < 0) {
                SwitchLights(REVERSELIGHTS, tTrue);
            } else {
                SwitchLights(REVERSELIGHTS, tFalse);
            }
        }

        if((_clock->GetStreamTime() - breakLightTimer) > breakTimeOffset) {
            lastSpeed = currentSpeed;
            breakLightTimer = _clock->GetStreamTime();
        }
    } else if (state == stateCar::stateCar_STARTUP) {
        ResetLights();
        SwitchLights(BREAKLIGHTS, tTrue);
    } else if (state == stateCar::stateCar_COMPLETE) {
        SwitchLights(BREAKLIGHTS, tTrue);
    } else if (state == stateCar::stateCar_READY) {
        SwitchLights(HEADLIGHTS, tTrue);
        SwitchLights(BREAKLIGHTS, tTrue);
    } else if (state == stateCar::stateCar_ERROR) {
        SwitchLights(HAZARDLIGHTS, tTrue);
    }

    TransmitStatus(BT::SUCCESS);
    RETURN_NOERROR;
}
