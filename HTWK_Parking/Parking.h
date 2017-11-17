#ifndef PARKING_HEADER
#define PARKING_HEADER

#include "stdafx.h"
#include "../HTWK_Map/Orientation.h"
#include "../HTWK_Types/IntersectionState.h"
#include "aadc_juryEnums.h"
#include "../HTWK_Behavior_Tree/HTWK_LeafNode/Leaf.h"
#include "../HTWK_Types/ParkingLot.h"
#include <ManeuverList.h>
#include <aadc_structs.h>
#include <iostream>
#include <HTWKPoint.hpp>
#include "../HTWK_Types/ManueverEnum.h"
#include <HTWKUtils.hpp>
#include "../HTWK_Debug/EnableLogs.h"

#define OID "htwk.Parking"
#define FILTER_NAME "HTWK BT:Node Parking"

#define LOT_DETECTION_DISTANCE 0.5
#define PARKING_WAIT_TIME 4000000
#define NINETY_DEG 1.57f

#define DRIVE_OUT_ANGLE_PROPERTY "drive out angle"
#define DRIVE_OUT_ANGLE_DEFAULT 0.5
#define DRIVE_OUT_ANGLE_DESCRIPTION "stop driving out when turned for this angle"
#define DRIVE_OUT_ANGLE_MIN 0.01
#define DRIVE_OUT_ANGLE_MAX 3.2

#define DRIVE_OUT_OFFSET_PROPERTY "drive out offset"
#define DRIVE_OUT_OFFSET_DEFAULT 0.09
#define DRIVE_OUT_OFFSET_DESCRIPTION "offset at which car sees angles as equal in Rad"
#define DRIVE_OUT_OFFSET_MIN 0
#define DRIVE_OUT_OFFSET_MAX 1

#define DRIVE_OUT_STEERING_PROPERTY "drive out steering"
#define DRIVE_OUT_STEERING_DEFAULT 20
#define DRIVE_OUT_STEERING_MIN 10
#define DRIVE_OUT_STEERING_MAX 30

#define DRIVE_IN_STEERING_PROPERTY "drive in steering"
#define DRIVE_IN_STEERING_DEFAULT (-30)
#define DRIVE_IN_STEERING_MIN (-30)
#define DRIVE_IN_STEERING_MAX (-10)

#define IN_LOT_DISTANCE_PROPERTY "lot in distance"
#define IN_LOT_DISTANCE_DEFAULT 0.5
#define IN_LOT_DISTANCE_DESCRIPTION "distance to drive into parking lot"
#define IN_LOT_DISTANCE_MIN 0.0
#define IN_LOT_DISTANCE_MAX 2.0

#define OUT_LOT_DISTANCE_PROPERTY "lot out distance"
#define OUT_LOT_DISTANCE_DEFAULT 0.2
#define OUT_LOT_DISTANCE_DESCRIPTION "distance to drive straight after parking"
#define OUT_LOT_DISTANCE_MIN 0.0
#define OUT_LOT_DISTANCE_MAX 2.0

#define OUT_LOT_DISTANCE_EXTRA_PROPERTY "lot out distance extra (turnout left)"
#define OUT_LOT_DISTANCE_EXTRA_DEFAULT 0.2
#define OUT_LOT_DISTANCE_EXTRA_DESCRIPTION "extra distance to drive straight while pulling out"
#define OUT_LOT_DISTANCE_EXTRA_MIN 0.0
#define OUT_LOT_DISTANCE_EXTRA_MAX 1.0

#define PARKING_SPEED_PROPERTY "PARKING_SPEED"
#define PARKING_SPEED_DEFAULT 0.7
#define PARKING_SPEED_DESCRIPTION "PARKING_SPEED for parking"
#define PARKING_SPEED_MIN 0.4
#define PARKING_SPEED_MAX 2.0


class Parking : public Leaf {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter)

public:
    Parking(const tChar *__info);

    virtual ~Parking();

    tResult Init(tInitStage eStage, __exception) override;

private:
    //property values
    tFloat32 driveOutAngle;
    tFloat32 driveOutOffsetInRad;
    int driveOutSteering;
    int driveInSteering;
    tFloat32 inLotDistance;
    tFloat32 outLotDistance;
    tFloat32 outLotDistanceExtra;
    tFloat32 speed;

    bool pullOutFirst = true;

    // Variables
    stateCar state;
    vector<ParkingLot> parkingLots;
    bool stopNow;
    tFloat32 headingOnStart;
    tTimeStamp stopTimer;
    enum Step {
        A_FORWARD_TURN, B_BACKWARD_TURN, C_BACKWARD_STRAIGHT, D_WAIT, E_PULL_OUT_STRAIGHT, F_PULL_OUT_TURN
    };
    Step step = A_FORWARD_TURN;
    HTWKPoint position;
    HTWKPoint lastPosition;

    tResult forwardTurn(tFloat32 headingInRad);             // Step A
    tResult backwardTurn(tFloat32 headingInRad);            // Step B
    tResult backwardStraight(ParkingLot targetParkingLot);  // Step C
    tResult wait();                                         // Step D

    tResult pullOutStraight(ManeuverEnum maneuver);   // Step E
    tResult pullOutTurnRight(tFloat32 headingInRad);             // Step F
    tResult pullOutTurnLeft(tFloat32 headingInRad);             // Step F

    tFloat32 addHeading(tFloat32 headingInRad, tFloat32 value);

    tFloat32 cleanHeading(tFloat32 headingInRad);

    tResult OnTrigger() override;

    tResult Shutdown(tInitStage eStage, IException **__exception_ptr) override;
};

#endif // PARKING_HEADER
