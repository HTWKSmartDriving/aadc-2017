#include "Parking.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, Parking)

Parking::Parking(const tChar *__info) : Leaf(__info) {
    stopNow = false;

    SetPropertyFloat(DRIVE_OUT_ANGLE_PROPERTY, DRIVE_OUT_ANGLE_DEFAULT);
    SetPropertyStr(DRIVE_OUT_ANGLE_PROPERTY NSSUBPROP_DESCRIPTION, DRIVE_OUT_ANGLE_DESCRIPTION);
    SetPropertyFloat(DRIVE_OUT_ANGLE_PROPERTY  NSSUBPROP_MIN, DRIVE_OUT_ANGLE_MIN);
    SetPropertyFloat(DRIVE_OUT_ANGLE_PROPERTY  NSSUBPROP_MAX, DRIVE_OUT_ANGLE_MAX);

    SetPropertyFloat(DRIVE_OUT_OFFSET_PROPERTY, DRIVE_OUT_OFFSET_DEFAULT);
    SetPropertyStr(DRIVE_OUT_OFFSET_PROPERTY NSSUBPROP_DESCRIPTION, DRIVE_OUT_OFFSET_DESCRIPTION);
    SetPropertyFloat(DRIVE_OUT_OFFSET_PROPERTY  NSSUBPROP_MIN, DRIVE_OUT_OFFSET_MIN);
    SetPropertyFloat(DRIVE_OUT_OFFSET_PROPERTY  NSSUBPROP_MAX, DRIVE_OUT_OFFSET_MAX);

    SetPropertyInt(DRIVE_OUT_STEERING_PROPERTY, DRIVE_OUT_STEERING_DEFAULT);
    SetPropertyInt(DRIVE_OUT_STEERING_PROPERTY  NSSUBPROP_MIN, DRIVE_OUT_STEERING_MIN);
    SetPropertyInt(DRIVE_OUT_STEERING_PROPERTY  NSSUBPROP_MAX, DRIVE_OUT_STEERING_MAX);

    SetPropertyInt(DRIVE_IN_STEERING_PROPERTY, DRIVE_IN_STEERING_DEFAULT);
    SetPropertyInt(DRIVE_IN_STEERING_PROPERTY  NSSUBPROP_MIN, DRIVE_IN_STEERING_MIN);
    SetPropertyInt(DRIVE_IN_STEERING_PROPERTY  NSSUBPROP_MAX, DRIVE_IN_STEERING_MAX);

    SetPropertyFloat(IN_LOT_DISTANCE_PROPERTY, IN_LOT_DISTANCE_DEFAULT);
    SetPropertyStr(IN_LOT_DISTANCE_PROPERTY NSSUBPROP_DESCRIPTION, IN_LOT_DISTANCE_DESCRIPTION);
    SetPropertyFloat(IN_LOT_DISTANCE_PROPERTY  NSSUBPROP_MIN, IN_LOT_DISTANCE_MIN);
    SetPropertyFloat(IN_LOT_DISTANCE_PROPERTY  NSSUBPROP_MAX, IN_LOT_DISTANCE_MAX);

    SetPropertyFloat(OUT_LOT_DISTANCE_PROPERTY, OUT_LOT_DISTANCE_DEFAULT);
    SetPropertyStr(OUT_LOT_DISTANCE_PROPERTY NSSUBPROP_DESCRIPTION, OUT_LOT_DISTANCE_DESCRIPTION);
    SetPropertyFloat(OUT_LOT_DISTANCE_PROPERTY  NSSUBPROP_MIN, OUT_LOT_DISTANCE_MIN);
    SetPropertyFloat(OUT_LOT_DISTANCE_PROPERTY  NSSUBPROP_MAX, OUT_LOT_DISTANCE_MAX);

    SetPropertyFloat(OUT_LOT_DISTANCE_EXTRA_PROPERTY, OUT_LOT_DISTANCE_EXTRA_DEFAULT);
    SetPropertyStr(OUT_LOT_DISTANCE_EXTRA_PROPERTY NSSUBPROP_DESCRIPTION, OUT_LOT_DISTANCE_EXTRA_DESCRIPTION);
    SetPropertyFloat(OUT_LOT_DISTANCE_EXTRA_PROPERTY  NSSUBPROP_MIN, OUT_LOT_DISTANCE_EXTRA_MIN);
    SetPropertyFloat(OUT_LOT_DISTANCE_EXTRA_PROPERTY  NSSUBPROP_MAX, OUT_LOT_DISTANCE_EXTRA_MAX);

    SetPropertyFloat(PARKING_SPEED_PROPERTY, PARKING_SPEED_DEFAULT);
    SetPropertyStr(PARKING_SPEED_PROPERTY NSSUBPROP_DESCRIPTION, PARKING_SPEED_DESCRIPTION);
    SetPropertyFloat(PARKING_SPEED_PROPERTY  NSSUBPROP_MIN, PARKING_SPEED_MIN);
    SetPropertyFloat(PARKING_SPEED_PROPERTY  NSSUBPROP_MAX, PARKING_SPEED_MAX);
}

Parking::~Parking() = default;

tResult Parking::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(Leaf::Init(eStage, __exception_ptr));

    if (StageNormal == eStage) {
        driveOutAngle = (tFloat32) GetPropertyFloat(DRIVE_OUT_ANGLE_PROPERTY, DRIVE_OUT_ANGLE_DEFAULT);
        driveOutOffsetInRad = static_cast<tFloat32>(GetPropertyFloat(DRIVE_OUT_OFFSET_PROPERTY, DRIVE_OUT_OFFSET_DEFAULT));
        driveOutSteering = GetPropertyInt(DRIVE_OUT_STEERING_PROPERTY, DRIVE_OUT_STEERING_DEFAULT);
        driveInSteering = GetPropertyInt(DRIVE_IN_STEERING_PROPERTY, DRIVE_IN_STEERING_DEFAULT);
        inLotDistance = (tFloat32) GetPropertyFloat(IN_LOT_DISTANCE_PROPERTY, IN_LOT_DISTANCE_DEFAULT);
        outLotDistance = (tFloat32) GetPropertyFloat(OUT_LOT_DISTANCE_PROPERTY, OUT_LOT_DISTANCE_DEFAULT);
        outLotDistanceExtra = (tFloat32) GetPropertyFloat(OUT_LOT_DISTANCE_EXTRA_PROPERTY, OUT_LOT_DISTANCE_EXTRA_DEFAULT);
        speed = (tFloat32) GetPropertyFloat(PARKING_SPEED_PROPERTY, PARKING_SPEED_DEFAULT);
    }

    RETURN_NOERROR;
}

tResult Parking::Shutdown(tInitStage eStage, __exception) {
    return Leaf::Shutdown(eStage, __exception_ptr);
}

tResult Parking::OnTrigger() {
    ManeuverEnum nextManeuver;
    tFloat32 headingInRad;
    int targetParkingLotId;
    ParkingLot targetParkingLot = ParkingLot();

    if (!IS_OK(worldService->Pull(WORLD_NEXT_MANEUVER, nextManeuver))) {
#ifdef DEBUG_PARKING_LOG
        LOG_ERROR("Next maneuver could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(WORLD_CURRENT_POSITION, position))) {
#ifdef DEBUG_PARKING_LOG
        LOG_ERROR("Current position could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(WORLD_CURRENT_HEADING, headingInRad))) {
#ifdef DEBUG_PARKING_LOG
        LOG_ERROR("Current heading could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(WORLD_PARKING_LOTS, parkingLots))) {
#ifdef DEBUG_PARKING_LOG
        LOG_ERROR("Parking lots could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(WORLD_TARGET_PARKING_LOT, targetParkingLotId))) {
#ifdef DEBUG_PARKING_LOG
        LOG_ERROR("Target parking lot could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(CAR_STATE, state))) {
#ifdef DEBUG_PARKING_LOG
        LOG_ERROR("Current car state could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if(state == stateCar::stateCar_RUNNING && nextManeuver == ManeuverEnum::PULLOUT_LEFT && pullOutFirst) {
        return pullOutTurnLeft(headingInRad);
    }

    if(nextManeuver == ManeuverEnum::CROSS_PARKING) {
        pullOutFirst = tTrue;
    }

    // Wenn der Fahrzeugstatus nicht RUNNING ist -> FAIL
    if (state != stateCar::stateCar_RUNNING) {
        TransmitStatus(BT::FAIL);
        RETURN_NOERROR;
    }

    // Wenn das nächste Maneuver nicht CROSS_PARKING ist -> FAIL
    if (!(nextManeuver == ManeuverEnum::CROSS_PARKING ||
          nextManeuver == ManeuverEnum::PULLOUT_LEFT ||
          nextManeuver == ManeuverEnum::PULLOUT_RIGHT)) {
        TransmitStatus(BT::FAIL);
        RETURN_NOERROR;
    }
    // Gehe die Liste aller bekannten Parkinglots und suche die ID, welche von der Jury empfangen wurde
    for (const auto &lot: parkingLots) {
        if (lot.id == targetParkingLotId) {
            targetParkingLot = lot;
        }
    }
#ifdef DEBUG_PARKING_LOG
    LOG_INFO(adtf_util::cString::Format("Nearing parking space! Parking space Position: x: %f y: %f",
                                        targetParkingLot.position.x(), targetParkingLot.position.y()));
    LOG_INFO(adtf_util::cString::Format("Current Position: x: %f y: %f", position.x(), position.y()));
#endif


    // BEGIN PARKING:


    if ((position.dist(targetParkingLot.position) < 3 * LOT_DETECTION_DISTANCE) && !stopNow &&
        nextManeuver == ManeuverEnum::CROSS_PARKING) {
        // start blinking
        SwitchLights(TURNSIGNAL_RIGHT, true);
        SetSpeedOutput(speed);
    }

    if ((position.dist(targetParkingLot.position) < LOT_DETECTION_DISTANCE) && !stopNow &&
        nextManeuver == ManeuverEnum::CROSS_PARKING) {
        headingOnStart = headingInRad;
        stopNow = true;
        // start blinking
        // SwitchLights(TURNSIGNAL_RIGHT, true);
    }

    // NOTE: all other calls of SwitchLights(...) are inside below functions to minimize the number of calls
    if (stopNow) {
        if (step == A_FORWARD_TURN) {
            return forwardTurn(headingInRad);
        } else if (step == B_BACKWARD_TURN) {
            return backwardTurn(headingInRad);
        } else if (step == C_BACKWARD_STRAIGHT) {
            return backwardStraight(targetParkingLot);
        } else if (step == D_WAIT) {
            return wait();
        }
    } else if (!stopNow &&
               (nextManeuver == ManeuverEnum::PULLOUT_RIGHT || nextManeuver == ManeuverEnum::PULLOUT_LEFT)) {
        if (step == E_PULL_OUT_STRAIGHT) {
            if (nextManeuver == ManeuverEnum::PULLOUT_RIGHT) {
                SwitchLights(TURNSIGNAL_RIGHT, true);
            } else if (nextManeuver == ManeuverEnum::PULLOUT_LEFT) {
                SwitchLights(TURNSIGNAL_LEFT, true);
            } else {
#ifdef DEBUG_PARKING_LOG
                LOG_ERROR("Maneuver after parking is invalid!");
#endif
                TransmitStatus(BT::FAIL);
                RETURN_ERROR(ERR_FAILED);
            }
            return pullOutStraight(nextManeuver);
        } else if (step == F_PULL_OUT_TURN) {
            if (nextManeuver == ManeuverEnum::PULLOUT_RIGHT) {
                return pullOutTurnRight(headingInRad);
            } else if (nextManeuver == ManeuverEnum::PULLOUT_LEFT) {
                return pullOutTurnLeft(headingInRad);
            } else {
#ifdef DEBUG_PARKING_LOG
                LOG_ERROR("Maneuver after parking is invalid!");
#endif
                TransmitStatus(BT::FAIL);
                RETURN_ERROR(ERR_FAILED);
            }
        }
    }

    TransmitStatus(BT::FAIL);
    RETURN_NOERROR;
}

// Step A
tResult Parking::forwardTurn(tFloat32 headingInRad) {
    tFloat32 turnOutHeading = addHeading(headingOnStart, driveOutAngle);
    tFloat32 cleanedHeading = 0;
    tFloat32 cleanedTurnOutHeading = 0;

    // check for overflow
    cleanedHeading = headingInRad < 0 && turnOutHeading > 0 ? static_cast<tFloat32>(headingInRad + 2 * M_PI) : headingInRad;
    cleanedTurnOutHeading = turnOutHeading < 0 && headingInRad > 0 ? static_cast<tFloat32>(turnOutHeading + 2 * M_PI) : turnOutHeading;

    if (!HTWKUtils::Equals(cleanedTurnOutHeading, cleanedHeading, driveOutOffsetInRad)) {
        SetSpeedOutput(speed);
        SetSteeringOutput(driveOutSteering);
#ifdef DEBUG_PARKING_LOG
        LOG_INFO("++ drive out ++ ");
        LOG_INFO(adtf_util::cString::Format("heading diff: %f", cleanedTurnOutHeading - cleanedHeading));
#endif
    } else {
#ifdef DEBUG_PARKING_LOG
        LOG_INFO("++ dont drive out ++ ");
        LOG_INFO(adtf_util::cString::Format("heading diff: %f", cleanedTurnOutHeading - cleanedHeading));
#endif
        SetSpeedOutput(0);
        SetSteeringOutput(0);
        step = B_BACKWARD_TURN;
    }

    TransmitStatus(BT::RUNNING);
    RETURN_NOERROR;
}

// Step B
tResult Parking::backwardTurn(tFloat32 headingInRad) {
    bool backwardTurn = false;
    tFloat32 turnOutHeading = addHeading(headingOnStart, NINETY_DEG);

    tFloat32 cleanedHeading = 0;
    tFloat32 cleanedTurnOutHeading = 0;

    // check for overflow
    cleanedHeading =
            headingInRad < 0 && turnOutHeading > 0 ? static_cast<tFloat32>(headingInRad + 2 * M_PI) : headingInRad;
    cleanedTurnOutHeading =
            turnOutHeading < 0 && headingInRad > 0 ? static_cast<tFloat32>(turnOutHeading + 2 * M_PI) : turnOutHeading;

    //stop turning if car turned for 90 degree
    if (!HTWKUtils::Equals(cleanedTurnOutHeading, cleanedHeading, driveOutOffsetInRad)) {
        backwardTurn = true;
#ifdef DEBUG_PARKING_LOG
        LOG_INFO(adtf_util::cString::Format("control heading: %f", turnOutHeading));
#endif
    }

    if (backwardTurn) {
#ifdef DEBUG_PARKING_LOG
        LOG_INFO(adtf_util::cString::Format("heading: %f", headingInRad));
#endif
        SetSpeedOutput(speed * -1);
        SetSteeringOutput(driveInSteering);
    } else {
        SetSteeringOutput(0);
        step = C_BACKWARD_STRAIGHT;
    }

    TransmitStatus(BT::RUNNING);
    RETURN_NOERROR;
}

// Step C
tResult Parking::backwardStraight(ParkingLot targetParkingLot) {
    if (position.dist(targetParkingLot.position) > inLotDistance) {
        lastPosition = position;
        stopTimer = _clock->GetStreamTime();
        step = D_WAIT;
        SetSpeedOutput(0);
        SetSteeringOutput(0);
        SwitchLights(TURNSIGNAL_RIGHT, false);
        SwitchLights(HAZARDLIGHTS, true);
    } else {
        SetSpeedOutput(speed * -1);
        SetSteeringOutput(0);
    }

    TransmitStatus(BT::RUNNING);
    RETURN_NOERROR;
}

// Step D
tResult Parking::wait() {
    if ((_clock->GetStreamTime() - stopTimer) > PARKING_WAIT_TIME) {
        stopNow = false;
        step = E_PULL_OUT_STRAIGHT;
        SwitchLights(HAZARDLIGHTS, false);
        IncrementManeuver();
    }

    TransmitStatus(BT::RUNNING);
    RETURN_NOERROR;
}

// Step E
tResult Parking::pullOutStraight(ManeuverEnum maneuver) {
    tFloat32 dist = (maneuver == ManeuverEnum::PULLOUT_LEFT) ? outLotDistance + outLotDistanceExtra : outLotDistance;
    if (position.dist(lastPosition) > dist) {
        step = F_PULL_OUT_TURN;
    } else {
#ifdef DEBUG_PARKING_LOG
        LOG_INFO(adtf_util::cString::Format("++ outLotDistance: %f", outLotDistance));
#endif
        SetSteeringOutput(0);
        SetSpeedOutput(speed);
    }

    TransmitStatus(BT::RUNNING);
    RETURN_NOERROR;
}

// Step F: Right
tResult Parking::pullOutTurnRight(tFloat32 headingInRad) {
    int steer = -30;
    tFloat32 cleanedHeading = cleanHeading(headingInRad);
    tFloat32 cleanedTurnHeading = cleanHeading(addHeading(headingOnStart, NINETY_DEG / 2));
    tFloat32 headingOnBegin = headingOnStart;

    if (headingInRad < 0 && headingOnBegin > 0) {
        SetSpeedOutput(speed);
        SetSteeringOutput(steer);
        TransmitStatus(BT::RUNNING);
    } else {
        if (cleanedHeading <= cleanedTurnHeading) {
            SwitchLights(TURNSIGNAL_RIGHT, false);
            SetSteeringOutput(0);
            IncrementManeuver();
            TransmitStatus(BT::FAIL); // quasi success
#ifdef DEBUG_PARKING_LOG
            LOG_INFO("++ parking finished");
#endif
        } else {
#ifdef DEBUG_PARKING_LOG
            LOG_INFO("turning right");
            LOG_INFO(adtf_util::cString::Format("++ ownHeading: %f targetHeading: %f ", cleanedHeading,
                                                cleanedTurnHeading));
#endif
            SetSpeedOutput(speed);
            SetSteeringOutput(steer);
            TransmitStatus(BT::RUNNING);
        }
    }
    RETURN_NOERROR;
}

// Step F: Left
tResult Parking::pullOutTurnLeft(tFloat32 headingInRad) {
    int steer = 30;
    tFloat32 turnHeading = addHeading(headingOnStart, static_cast<tFloat32>(-M_PI));

    if (headingInRad < 0 && turnHeading > 0) {
        SetSpeedOutput(speed);
        SetSteeringOutput(steer);
        TransmitStatus(BT::RUNNING);
    } else {
        if (cleanHeading(headingInRad) >= cleanHeading(turnHeading)) {
            SwitchLights(TURNSIGNAL_LEFT, false);
            SetSteeringOutput(0);
            IncrementManeuver();
            TransmitStatus(BT::FAIL); // quasi success
#ifdef DEBUG_PARKING_LOG
            LOG_INFO("++ parking finished");
#endif
        } else {
#ifdef DEBUG_PARKING_LOG
            LOG_INFO("turning left");
            LOG_INFO(adtf_util::cString::Format("++ ownHeading: %f targetHeading: %f ", headingInRad,
                                                addHeading(headingOnStart, NINETY_DEG)));
#endif
            SetSpeedOutput(speed);
            SetSteeringOutput(steer);
            TransmitStatus(BT::RUNNING);
        }
    }

    RETURN_NOERROR;
}


tFloat32 Parking::addHeading(tFloat32 headingInRad, tFloat32 value) {
    tFloat32 returnValue = headingInRad + value;
    if (headingInRad > M_PI / 2) {
        returnValue = static_cast<tFloat32>(returnValue - 2 * M_PI);
    }
    return returnValue;
}


tFloat32 Parking::cleanHeading(tFloat32 headingInRad) {
    if (headingInRad < 0) {
        return static_cast<tFloat32>(headingInRad + 2 * M_PI);
    } else {
        return headingInRad;
    }
}