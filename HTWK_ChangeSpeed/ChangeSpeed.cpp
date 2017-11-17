#include "ChangeSpeed.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, ChangeSpeed);

ChangeSpeed::ChangeSpeed(const tChar *__info) : Leaf(__info) {
    // speed properties
    SetPropertyFloat(ROAD_SPEED_PROPERTY, SPEED_ROAD);
    SetPropertyStr(ROAD_SPEED_PROPERTY NSSUBPROP_DESCRIPTION, "Speed to drive on road");

    SetPropertyFloat(INTERSECTION_SPEED_PROPERTY, SPEED_INTERSECTION);
    SetPropertyStr(INTERSECTION_SPEED_PROPERTY NSSUBPROP_DESCRIPTION, "Speed to drive on intersections");

    SetPropertyFloat(NEARING_GIVE_WAY_SPEED_PROPERTY, SPEED_NEARING_GIVE_WAY);
    SetPropertyStr(NEARING_GIVE_WAY_SPEED_PROPERTY NSSUBPROP_DESCRIPTION,
                   "Speed to drive when nearing give way intersection");

    SetPropertyFloat(NEARING_HAVE_WAY_SPEED_PROPERTY, SPEED_NEARING_HAVE_WAY);
    SetPropertyStr(NEARING_HAVE_WAY_SPEED_PROPERTY NSSUBPROP_DESCRIPTION,
                   "Speed to drive when nearing have way intersection");

    SetPropertyFloat(NEARING_FULL_STOP_SPEED_PROPERTY, SPEED_NEARING_FULL_STOP);
    SetPropertyStr(NEARING_FULL_STOP_SPEED_PROPERTY NSSUBPROP_DESCRIPTION,
                   "Speed to drive when nearing full stop intersection");

    SetPropertyFloat(NEARING_GENERIC_SPEED_PROPERTY, SPEED_NEARING_GENERIC);
    SetPropertyStr(NEARING_GENERIC_SPEED_PROPERTY NSSUBPROP_DESCRIPTION,
                   "Speed to drive when nearing generic intersection");




    //other properties
    SetPropertyFloat(DETECT_DIST_PROPERTY, DETECT_DIST_DEFAULT);
    SetPropertyStr(DETECT_DIST_PROPERTY NSSUBPROP_DESCRIPTION, DETECT_DIST_DESCRIPTION);

    SetPropertyFloat(SLOW_SPEED_PROPERTY, SLOW_SPEED_DEFAULT);
    SetPropertyStr(SLOW_SPEED_PROPERTY NSSUBPROP_DESCRIPTION, SLOW_SPEED_DESCRIPTION);

    SetPropertyFloat(STOP_AT_PERSON_PROPERTY, STOP_AT_PERSON_DEFAULT);
    SetPropertyStr(STOP_AT_PERSON_PROPERTY NSSUBPROP_DESCRIPTION, STOP_AT_PERSON_DESCRIPTION);

    SetPropertyFloat(LANE_WIDTH_PROPERTY, LANE_WIDTH_DEFAULT);

    SetPropertyFloat(ROAD_OBTACLE_DIST_PROPERTY, ROAD_OBTACLE_DIST_DEFAULT);
    SetPropertyFloat(INTERSECTION_OBTACLE_DIST_PROPERTY, INTERSECTION_OBTACLE_DIST_DEFAULT);
    SetPropertyFloat(PEDESTRIAN_OBTACLE_DIST_PROPERTY, PEDESTRIAN_OBTACLE_DIST_DEFAULT);



}

ChangeSpeed::~ChangeSpeed()
= default;

tResult ChangeSpeed::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(Leaf::Init(eStage, __exception_ptr));
    if (eStage == StageNormal) {
        detectDist = static_cast<tFloat32>(GetPropertyFloat(DETECT_DIST_PROPERTY, DETECT_DIST_DEFAULT));
        slowSpeed = static_cast<tFloat32>(GetPropertyFloat(SLOW_SPEED_PROPERTY, SLOW_SPEED_DEFAULT));

        speedOnRoad = static_cast<tFloat32>(GetPropertyFloat(ROAD_SPEED_PROPERTY));
        speedNearingHaveWay = static_cast<tFloat32>(GetPropertyFloat(NEARING_HAVE_WAY_SPEED_PROPERTY));
        speedNearingGiveWay = static_cast<tFloat32>(GetPropertyFloat(NEARING_GIVE_WAY_SPEED_PROPERTY));
        speedNearingFullStop = static_cast<tFloat32>(GetPropertyFloat(NEARING_FULL_STOP_SPEED_PROPERTY));
        speedNearingGeneric = static_cast<tFloat32>(GetPropertyFloat(NEARING_GENERIC_SPEED_PROPERTY));
        speedIntersection = static_cast<tFloat32>(GetPropertyFloat(INTERSECTION_SPEED_PROPERTY));
        stopPersonTime = static_cast<int>(GetPropertyFloat(STOP_AT_PERSON_PROPERTY, STOP_AT_PERSON_DEFAULT) * 1000000);
        laneWidth = GetPropertyFloat(LANE_WIDTH_PROPERTY, LANE_WIDTH_DEFAULT);
        distRoad = GetPropertyFloat(ROAD_OBTACLE_DIST_PROPERTY, ROAD_OBTACLE_DIST_DEFAULT);
        distPedestrian = GetPropertyFloat(PEDESTRIAN_OBTACLE_DIST_PROPERTY, PEDESTRIAN_OBTACLE_DIST_DEFAULT);
        distIntersection = GetPropertyFloat(INTERSECTION_OBTACLE_DIST_PROPERTY, INTERSECTION_OBTACLE_DIST_DEFAULT);
        pedestrianTime = 5000000;
    }
    RETURN_NOERROR;
}

tResult ChangeSpeed::OnTrigger() {
    std::vector<tTrackingData> allObstacles;

    IntersectionState intersectionState;
    if (!IS_OK(worldService->Pull(WORLD_CURRENT_POSITION, currentPos))) {
//        LOG_ERROR("Current Position could not be pulled!");
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }
    if (!IS_OK(worldService->Pull(WORLD_OBSTACLES, allObstacles))) {
        noObstacles = true;
    } else {
        noObstacles = false;
    }
    if (!IS_OK(worldService->Pull(WORLD_CURRENT_HEADING, heading))) {
//        LOG_ERROR("Heading could not be pulled!");
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }
//    if (!IS_OK(worldService->Pull(WORLD_ZEBRA_STATE, zebraState))) {
//        LOG_ERROR("Zebra State could not be pulled!");
//        RETURN_ERROR(ERR_FAILED);
//    }
    if (!IS_OK(worldService->Pull(WORLD_INTERSECTION_STATE, intersectionState))) {
//        LOG_ERROR("intersection state could not be pulled!");
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(WORLD_CURRENT_SPEED, currentSpeed))) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_ERROR("Current speed could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(CAR_STATE, state))) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_ERROR("State could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(WORLD_NEXT_TURN, nextTurn))) {
//        LOG_ERROR("nextTurn could not be pulled!");
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(WORLD_ROAD_SIGN_EXT, nextSign))) {
//        LOG_ERROR("road sign could not be pulled!");
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }


//    heading = HTWKMathUtils::rad2deg(heading);

//    if (!IS_OK(worldService->Pull(WORLD_LANE_LEFT, leftLaneDist))) {
    leftLaneDist = laneWidth * 1.5;
//    }
//    if (!IS_OK(worldService->Pull(WORLD_LANE_RIGHT, rightLaneDist))) {
    rightLaneDist = laneWidth / 2;
//    }

    return findSpeedNew(allObstacles, intersectionState);
}

//      90
//      |
//180 -- -- 0
//      |
//     -90
//HTWKLane
//ChangeSpeed::ExtractLanesFromPositionAndHeading(HTWKPoint carPos, tFloat carHeading) {
//    HTWKLane lane;
//    // berechne Punkt der auf der Fahrspur liegt
//    // linke Spur: berechne Vektor der 90 Grad nach links von aktueller Position aus zeigt, mit Länge der Distanz zur Spur
//    // rechte Spur: Vektor zeigt 90 Grad nach rechts
//    tFloat rotatedHeading = carHeading + 90;
//    //rechne von Polarkoordinaten in kartesische um
//    HTWKPoint vector2Lane = PolarToCartesian(rightLaneDist, rotatedHeading);
//    // addiere berechneten Vektor auf die eigene Position = Spurpunkt 1
//    lane.rightStart = carPos.add(vector2Lane);
//    // berechne 2. Spurpunkt, durch Addition eines Vektors mit beliebiger Länge und Winkel in Richtung des Heading des Autos
//    // rechne in kartesische Koordinaten um
//    HTWKPoint lanePoint2 = lane.rightStart.add(PolarToCartesian(1.0, carHeading));
//    // lege Gerade durch beide Punkte -> ~Spur
//    lane.right = HTWKPoint(lanePoint2.x() - lane.rightStart.x(), lanePoint2.y() - lane.rightStart.y());
//
//    // das gleiche für die mittlere Spur
//    // Vektor auf die Spur zeigt 90 Grad nach links
//    rotatedHeading = carHeading - 90;
//    //dist zur Mitte = dist nach rechts
//    vector2Lane = PolarToCartesian(rightLaneDist, rotatedHeading);
//    lane.middleStart = carPos.add(vector2Lane);
//    lanePoint2 = lane.middleStart.add(PolarToCartesian(1.0, carHeading));
//    lane.middle = HTWKPoint(lanePoint2.x() - lane.middleStart.x(), lanePoint2.y() - lane.middleStart.y());
//
//    vector2Lane = PolarToCartesian(leftLaneDist, rotatedHeading);
//    lane.leftStart = carPos.add(vector2Lane);
//    lanePoint2 = lane.leftStart.add(PolarToCartesian(1.0, carHeading));
//    lane.left = HTWKPoint(lanePoint2.x() - lane.leftStart.x(), lanePoint2.y() - lane.leftStart.y());
//    return lane;
//}


// x = r * cos (alpha)
// y = r * sin (alpha)
//HTWKPoint ChangeSpeed::PolarToCartesian(tFloat length, tFloat angle) {
//    double x = length * std::cos(angle);
//    double y = length * std::sin(angle);
//    return HTWKPoint(x, y);
//}


ObstaclePos ChangeSpeed::IsPositionOnRoad(cv::Point2f obstaclePosition) {
    cv::Point2f carPosition;
    carPosition.x = static_cast<float>(-currentPos.x());
    carPosition.y = static_cast<float>(-currentPos.y());
    cv::Point2f newObstPos = htwk::translateAndRotate2DPoint(obstaclePosition,
                                                             static_cast<float>(-(heading)),
                                                             carPosition);

//    std::string log = "x: " + to_string(newObstPos.x) + " y: " + to_string(newObstPos.y);
//    LOG_INFO(log.c_str());
    if (newObstPos.x >= 0) {
        //rechte Seite
        if (newObstPos.x <= rightLaneDist) {
            return ObstaclePos::LANE;
        } else {
            return ObstaclePos::OFFROAD_R;
        }
    } else {
        if (-newObstPos.x <= rightLaneDist) {
            return ObstaclePos::LANE;
        } else if (-newObstPos.x <= leftLaneDist) {
            return ObstaclePos::OTHER_LANE;
        } else {
            return ObstaclePos::OFFROAD_L;
        }
    }
}


tResult ChangeSpeed::FullBrake() {
//    LOG_INFO("do full brake");
    //stopPersonTimer = _clock->GetStreamTime() + stopPersonTime;
    SetSpeedOutput(0);
    return TransmitStatus(BT::SUCCESS);
}

tResult ChangeSpeed::findUsualSpeed(const IntersectionState &interState) {
    tFloat32 speed = 0;

    if (state == stateCar::stateCar_RUNNING) {
        switch (interState) {
            case IntersectionState::ON_ROAD:
#ifdef DEBUG_MAP_VIEW_LOG
                LOG_INFO("IntersectionState: ON_ROAD");
#endif
                speed = speedOnRoad;
                fullStopDone = false;
                stopTimer = 0;
                break;
            case IntersectionState::NEARING_HAVE_WAY:
#ifdef DEBUG_MAP_VIEW_LOG
                LOG_INFO("IntersectionState: NEARING_HAVE_WAY");
#endif
                speed = speedNearingHaveWay;
                fullStopDone = false;
                stopTimer = 0;
                break;
            case IntersectionState::NEARING_GIVE_WAY:
#ifdef DEBUG_MAP_VIEW_LOG
                LOG_INFO("IntersectionState: NEARING_GIVE_WAY");
#endif
                speed = speedNearingGiveWay;
                fullStopDone = false;
                stopTimer = 0;
                break;
            case IntersectionState::NEARING_FULL_STOP:
#ifdef DEBUG_MAP_VIEW_LOG
                LOG_INFO("IntersectionState: NEARING_FULL_STOP");
#endif
                speed = speedNearingFullStop;
                fullStopDone = false;
                stopTimer = 0;
                break;
            case IntersectionState::ON_INTERSECTION:
#ifdef DEBUG_MAP_VIEW_LOG
                LOG_INFO("IntersectionState: ON_INTERSECTION");
#endif
                speed = speedIntersection;
                break;
            case IntersectionState::STOP_NOW:
#ifdef DEBUG_MAP_VIEW_LOG
                LOG_INFO("IntersectionState: STOP_NOW");
#endif
                if (!fullStopDone) {
                    if (currentSpeed > 0.1) {
                        speed = 0;
                    } else {
                        if (stopTimer == 0) {
                            stopTimer = _clock->GetStreamTime() + STOP_AT_STOP_SIGN;
                        } else if (_clock->GetStreamTime() > stopTimer) {
                            fullStopDone = tTrue;
                        }
                    }

                } else {
                    speed = speedIntersection;
                }

                break;
            case IntersectionState::NEARING_GENERIC:
#ifdef DEBUG_MAP_VIEW_LOG
                LOG_INFO("IntersectionState: NEARING_GENERIC");
#endif
                speed = speedNearingGeneric;
                fullStopDone = false;
                stopTimer = 0;
                break;
        }
    } else {
        speed = 0;
    }

    SetSpeedOutput(speed);

    RETURN_NOERROR;
}

tResult ChangeSpeed::findSpeedNew(const vector<tTrackingData> &obstacles, const IntersectionState &interState) {


    //pedestrian
    if (pedestrianTimer != 0) {
        SetSpeedOutput(0);
        if (pedestrianTimer + pedestrianTime > _clock->GetStreamTime()) {
            pedestrianTimer = 0;
        }
        return TransmitStatus(BT::SUCCESS);
    }

    HTWKPoint pedestrianPos1 = HTWKPoint(3.5, 0.5);
    HTWKPoint pedestrianPos2 = HTWKPoint(3.5, 10.5);
    if (HTWKUtils::Equals(pedestrianPos1, currentPos, distPedestrian) || HTWKUtils::Equals(pedestrianPos2, currentPos, distPedestrian)) {
        SetSpeedOutput(0);
        pedestrianTimer = _clock->GetStreamTime();
        return TransmitStatus(BT::SUCCESS);
    }

    if (interState == IntersectionState::STOP_NOW) {
        if (!IS_OK(findObstacleSpeedOnIntersection(obstacles))) {
            findUsualSpeed(interState);
            return TransmitStatus(BT::SUCCESS);
        } else {
            RETURN_NOERROR;
        }
    }

    if (!IS_OK(findObstacleSpeedOnRoad(obstacles))) {
        findUsualSpeed(interState);
        return TransmitStatus(BT::SUCCESS);
    } else {
        RETURN_NOERROR;
    }
}


tResult ChangeSpeed::findObstacleSpeedOnRoad(const vector<tTrackingData> &obstacles) {

    if (stopPersonTimer != 0) {
        SetSpeedOutput(slowSpeed);
        if (stopPersonTimer + stopPersonTime < _clock->GetStreamTime()) {
            stopPersonTimer = 0;
        }
        RETURN_ERROR(ERR_FAILED);
    }
    if (noObstacles) {
        RETURN_ERROR(ERR_FAILED);
    }
    for (const tTrackingData &obstacle : obstacles) {
        HTWKPoint distPoint;
        distPoint.setXval(obstacle.position.x);
        distPoint.setYval(obstacle.position.y);
        if (obstacle.type == ObstacleType::CHILD &&
            distPoint.dist(currentPos) <= distRoad) {
            stopPersonTimer = _clock->GetStreamTime();
            SetSpeedOutput(slowSpeed);
            return TransmitStatus(BT::SUCCESS);
        }
    }
    RETURN_ERROR(ERR_FAILED);

}


tResult
ChangeSpeed::findObstacleSpeedOnIntersection(const vector<tTrackingData> &obstacles) {
    if (noObstacles) {
        RETURN_ERROR(ERR_FAILED);
    }
    for (tTrackingData obstacle : obstacles) {
        ObstaclePos obstRoad = IsPositionOnRoad(obstacle.position);
        HTWKPoint distPoint;
        distPoint.setXval(obstacle.position.x);
        distPoint.setYval(obstacle.position.y);
        if (obstacle.type == ObstacleType::CAR
            && distPoint.dist(currentPos) < distIntersection) {
            switch (nextTurn) {
                case Orientation::TurnDirection::STRAIGHT:
                    if (obstRoad == ObstaclePos::OFFROAD_R) {
                        return FullBrake();
                    } else if (obstRoad == ObstaclePos::OFFROAD_L) {
                        return FullBrake();
                    } else {
                        RETURN_ERROR(ERR_FAILED);
                    }
                case Orientation::TurnDirection::LEFT: {
                    return FullBrake();
                }
                case Orientation::TurnDirection::RIGHT:
                    if (obstRoad == ObstaclePos::OFFROAD_L) {
                        return FullBrake();
                    } else {
                        RETURN_ERROR(ERR_FAILED);
                    }
            }
        }
    }
    RETURN_ERROR(ERR_FAILED);
}

//tResult ChangeSpeed::findObstacleSpeedOnPedestrian(const vector<tTrackingData> &obstacles) {
//    if (noObstacles) {
//        RETURN_ERROR(ERR_FAILED);
//    }
//
//    ObstaclePos obstRoad;
//    HTWKPoint distPoint;
//    bool slowDown = false;
//
//    for (tTrackingData obstacle : obstacles) {
//        obstRoad = IsPositionOnRoad(obstacle.position);
//        distPoint.setXval(obstacle.position.x);
//        distPoint.setYval(obstacle.position.y);
//        if(distPoint.dist(currentPos) > distPedestrian){
//            RETURN_ERROR(ERR_FAILED);
//        }
//
//        if (obstacle.type == ObstacleType::ADULT || obstacle.type == ObstacleType::CHILD) {
//
//            if (firstObstacleSight) {
//                firstObstacleSight = false;
//                firstObstaclePos = obstRoad;
//                if (currentPos.dist(distPoint) <= pedestrianDetectDist) {
//                    slowDown = true;
//                } else {
//                    RETURN_ERROR(ERR_FAILED);
//                }
//            } else {
//                if (obstRoad == ObstaclePos::LANE || obstRoad == ObstaclePos::OTHER_LANE) {
//                    return FullBrake();
//                } else if (obstRoad == firstObstaclePos) {
//                    return FullBrake();
//                } else {
//                    firstObstacleSight = true;
//                    RETURN_ERROR(ERR_FAILED);
//                }
//            }
//        }
//    }
//    if (slowDown) {
//        SetSpeedOutput(slowSpeed);
//        return TransmitStatus(BT::SUCCESS);
//    } else {
//        RETURN_ERROR(ERR_FAILED);
//    }
//}


//tFloat ChangeSpeed::VectorAngle(HTWKPoint vec1, HTWKPoint vec2) {
//    tFloat prod = vec1.x() * vec2.x() + vec1.y() * vec2.y();
//    tFloat length1 = std::sqrt(vec1.x() * vec1.x() + vec1.y() * vec1.y());
//    tFloat length2 = std::sqrt(vec2.x() * vec2.x() + vec2.y() * vec2.y());
//    return prod / (length1 * length2);
//}