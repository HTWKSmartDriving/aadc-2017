#include "WorldOutTemplateFilter.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, WorldOutTemplateFilter)

WorldOutTemplateFilter::WorldOutTemplateFilter(const tChar *__info) : WorldOutBaseFilter(__info) {

}

WorldOutTemplateFilter::~WorldOutTemplateFilter()
= default;

tResult WorldOutTemplateFilter::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(WorldOutBaseFilter::Init(eStage, __exception_ptr));

    if (StageFirst == eStage) {

    } else if (StageNormal == eStage) {

    } else if (StageGraphReady == eStage) {

    }

    RETURN_NOERROR;
}

tResult WorldOutTemplateFilter::Shutdown(tInitStage eStage, __exception) {
    return WorldOutBaseFilter::Shutdown(eStage, __exception_ptr);
}

tResult WorldOutTemplateFilter::OnTrigger(tFloat32 interval) {
    //some examples how to access data from world service
    //pls see wiki for further informations on used types

    //read current speed value
    tFloat32 speed;
    if (IS_OK(worldService->Pull(WORLD_CURRENT_SPEED, speed))) {
        std::cout << "Current Speed " << speed << std::endl;
    }

    //read current action
    juryActions action;
    if (IS_OK(worldService->Pull(WORLD_ACTION_ID, action))) {
        if (action == juryActions::action_GETREADY) {
            std::cout << "Action: GetReady" << std::endl;
        }
    }

    //read current maneuver
    tInt16 maneuver;
    if (IS_OK(worldService->Pull(CAR_MANEUVER_ENTRY, maneuver))) {
        std::cout << "Current Manuever " << maneuver << std::endl;
    }

    RETURN_NOERROR;
}