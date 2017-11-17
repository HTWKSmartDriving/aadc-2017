#include "JuryResponse.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, JuryResponse)

JuryResponse::JuryResponse(const tChar *__info) : Leaf(__info) {
    readyUpTimer = 0;
}

JuryResponse::~JuryResponse()
= default;

tResult JuryResponse::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(Leaf::Init(eStage, __exception_ptr));

    if (StageFirst == eStage) {
        //load tile images for visualization
    } else if (StageNormal == eStage) {

    } else if (StageGraphReady == eStage) {

    }

    RETURN_NOERROR;
}

tResult JuryResponse::Shutdown(tInitStage eStage, __exception) {
    return Leaf::Shutdown(eStage, __exception_ptr);
}

tResult JuryResponse::OnTrigger() {
    tFloat32 currentSpeed;
    bool speedCouldBePulled = false;
    juryActions action;
    tInt16 maneuverEntry;
    tInt16 lastManeuverEntryInList;

    if (!IS_OK(worldService->Pull(WORLD_ACTION_ID, action))) {
#ifdef DEBUG_JURY_RESPONSE_LOG
        LOG_ERROR("Action ID could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(CAR_MANEUVER_ENTRY, maneuverEntry))) {
#ifdef DEBUG_JURY_RESPONSE_LOG
        LOG_ERROR("ManeuverEntry could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(WORLD_LAST_MANEUVER_ENTRY_IN_LIST, lastManeuverEntryInList))) {
#ifdef DEBUG_JURY_RESPONSE_LOG
        LOG_ERROR("lastManeuverEntryInList could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(WORLD_CURRENT_SPEED, currentSpeed))) {
#ifdef DEBUG_JURY_RESPONSE_LOG
        LOG_ERROR("Current speed could not be pulled from WorldService");
#endif
    } else {
        speedCouldBePulled = true;
    }


    if(state != stateCar::stateCar_COMPLETE) {
        if(maneuverEntry > lastManeuverEntryInList)
        {
            state = stateCar::stateCar_COMPLETE;
            SetCarState(state);
            return TransmitStatus(BT::SUCCESS);
        }

        switch (action) {
            case action_STOP:
                state = stateCar::stateCar_ERROR;
                SetCarState(state);
                return TransmitStatus(BT::SUCCESS);
            case action_GETREADY:
                if (readyUpTimer == 0) {
                    readyUpTimer = _clock->GetStreamTime();
                } else if ((_clock->GetStreamTime() - readyUpTimer) < READY_UP_WAIT_TIME) {
                    state = stateCar::stateCar_STARTUP;
                    SetCarState(state);
                } else {
                    if (speedCouldBePulled) {
                        state = stateCar::stateCar_READY;
                        SetCarState(state);
                    }
                }
                break;
            case action_START:
                if (state == stateCar::stateCar_RUNNING) {
                    TransmitStatus(BT::SUCCESS);
                    RETURN_NOERROR;
                } else if (state == stateCar::stateCar_READY) {
                    readyUpTimer = 0;
                    state = stateCar::stateCar_RUNNING;
                    SetCarState(state);
                } else {
#ifdef DEBUG_JURY_RESPONSE_LOG
                    LOG_INFO("Car not ready yet.");
#endif
                }
                break;
        }
    }

    TransmitStatus(BT::FAIL);
    RETURN_NOERROR;
}
