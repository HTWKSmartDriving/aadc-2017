#include "UsObstacleDetection.h"
#include "../HTWK_Utils/HTWKUtils.hpp"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, UsObstacleDetection);

UsObstacleDetection::UsObstacleDetection(const tChar *__info) : Leaf(__info) {

//Property for distance, in which obstacles get ignored (correcting false positives of ultrasonic sensors)
    SetPropertyFloat(MIN_US_DISTANCE_PROPERTY, MIN_US_DISTANCE_DEFAULT);
    SetPropertyStr(MIN_US_DISTANCE_PROPERTY NSSUBPROP_DESCRIPTION, MIN_US_DISTANCE_DESCRIPTION);
    SetPropertyFloat(MIN_US_DISTANCE_PROPERTY  NSSUBPROP_MIN, MIN_US_DISTANCE_MIN);
    SetPropertyFloat(MIN_US_DISTANCE_PROPERTY  NSSUBPROP_MAX, MIN_US_DISTANCE_MAX);

    //Property for minimum distance, at which car starts braking
    SetPropertyFloat(MIN_BRAKE_DISTANCE_PROPERTY, MIN_BRAKE_DISTANCE_DEFAULT);
    SetPropertyStr(MIN_BRAKE_DISTANCE_PROPERTY NSSUBPROP_DESCRIPTION, MIN_BRAKE_DISTANCE_DESCRIPTION);
    SetPropertyFloat(MIN_BRAKE_DISTANCE_PROPERTY  NSSUBPROP_MIN, MIN_BRAKE_DISTANCE_MIN);
    SetPropertyFloat(MIN_BRAKE_DISTANCE_PROPERTY  NSSUBPROP_MAX, MIN_BRAKE_DISTANCE_MAX);
}

UsObstacleDetection::~UsObstacleDetection()
= default;

tResult UsObstacleDetection::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(Leaf::Init(eStage, __exception_ptr));
    if (eStage == StageFirst) {

        donotBrakeDistancePropValue = GetPropertyFloat(MIN_US_DISTANCE_PROPERTY);
        minBrakeDistancePropValue = GetPropertyInt(MIN_BRAKE_DISTANCE_PROPERTY);
    }

    RETURN_NOERROR;
}

tResult UsObstacleDetection::OnTrigger() {
    if (!IS_OK(worldService->Pull(WORLD_CURRENT_SPEED, currentSpeed))) {
        TransmitStatus(BT::FAIL);
#ifdef DEBUG_OBSTACLE_DETECTION_LOG
        RETURN_AND_LOG_ERROR_STR(ERR_FAILED, "Current speed could not be pulled from WorldService");
#endif
    }

    if (!IS_OK(worldService->Pull(WORLD_ULTRASONICS, ultrasonicStruct))) {
        TransmitStatus(BT::FAIL);
#ifdef DEBUG_OBSTACLE_DETECTION_LOG
        RETURN_AND_LOG_ERROR_STR(ERR_FAILED, "US Struct could not be pulled from WorldService");
#endif
    }

    if (&ultrasonicStruct != NULL) {
        if (DrivePossible(currentSpeed, ultrasonicStruct)) {
#ifdef DEBUG_OBSTACLE_DETECTION_LOG
            LOG_INFO("no obstacles, drive");
#endif
            worldService->Push(CAR_AVOID_MOVE, false);
            TransmitStatus(BT::FAIL);
        } else {
            TransmitStatus(BT::SUCCESS);
#ifdef DEBUG_OBSTACLE_DETECTION_LOG
            LOG_INFO("ObstacleDetection active and blocking!");
#endif
        }
    }

    // if (brake) TransmitStatus(BT::SUCCESS) else TransmitStatus(BT::FAIL)

    RETURN_NOERROR;
}

tFloat64 UsObstacleDetection::BrakingDistance(const tFloat32 &speed) {
    // braking formula is empirical tested and developed
    // made some tests with the braking distance of the model car
    // calculated trend formula from the values
    // formula returns value, which is greater than the real braking distance to avoid collisions
    // additionally add the MIN_US_PROPERTY * 2, to be in a range where the Ultasonic sensors detect obstacles
    return BRAKE_SLOPE_SQUARE * (speed * speed) + BRAKE_SLOPE_LINEAR * std::abs(speed) + minBrakeDistancePropValue +
           2 * donotBrakeDistancePropValue;
}


/// check if no obstacle is in front
/// return true if way is clear
bool UsObstacleDetection::checkFront(const tUltrasonicStruct &ultrasonicStruct, const tFloat64 &checkValue) {
    return (ultrasonicStruct.tFrontCenterRight.f32Value > checkValue &&
            ultrasonicStruct.tFrontCenter.f32Value > checkValue &&
            ultrasonicStruct.tFrontCenterLeft.f32Value > checkValue);
};


bool UsObstacleDetection::FrontFree(const tUltrasonicStruct &ultrasonicStruct, const tFloat32 &brakingDistance) {
    if (checkFront(ultrasonicStruct, donotBrakeDistancePropValue)) {
        return checkFront(ultrasonicStruct, brakingDistance);
    } else {
        // return true, if ultrasonic sensor values < MIN_US_DISTANCE_PROPERTY
        // removes false positives
        // should not make trouble because if car gets closer as 9 cm it starts braking,
        // car should never come this close and ultrasonic sensor values may be wrong at this distance
        return true;
    }
}

///     check if no obstacle is in back
///     return true if way is clear
bool UsObstacleDetection::checkBack(const tUltrasonicStruct &ultrasonicStruct, const tFloat64 &checkValue) {
    return ultrasonicStruct.tRearRight.f32Value > checkValue &&
           ultrasonicStruct.tRearCenter.f32Value > checkValue &&
           ultrasonicStruct.tRearLeft.f32Value > checkValue;
};


bool UsObstacleDetection::BackFree(const tUltrasonicStruct &ultrasonicStruct, const tFloat32 &brakingDistance) {


    if (checkBack(ultrasonicStruct, donotBrakeDistancePropValue)) {
        return checkBack(ultrasonicStruct, brakingDistance);
    } else {
        // same as at UsObstacleDetection::FrontFree()
        return true;
    }
}


bool
UsObstacleDetection::DrivePossible(const tFloat32 &drivenSpeed,
                                   const tUltrasonicStruct &ultrasonicStruct) {

    if (_clock->GetStreamTime() < timerForStop) {
        return false;
    } else {
        timerForStop = 0;
    }

    tFloat32 brakingDistance = BrakingDistance(drivenSpeed);
//        check if driving forward
    if (drivenSpeed > 0.1) {
        bool check = FrontFree(ultrasonicStruct, brakingDistance);
        if (!check) {
            timerForStop = _clock->GetStreamTime() + MIN_STOP_TIME;
        }
        return check;
//        check if driving backward
    } else if (drivenSpeed < -0.1) { //drive backward
        return true;
    } else {
        //TODO ändern auf return false, sobald Kameraerkennung danach möglich
        bool check = FrontFree(ultrasonicStruct, brakingDistance);
        if (!check) {
            timerForStop = _clock->GetStreamTime() + MIN_STOP_TIME;
        }
        return check;
    }

}
