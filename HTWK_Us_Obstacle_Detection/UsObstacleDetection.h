#ifndef HTWK_US_OBSTACLE_DETECTION
#define HTWK_US_OBSTACLE_DETECTION

#include "stdafx.h"
#include "../HTWK_Behavior_Tree/HTWK_LeafNode/Leaf.h"
#include "../HTWK_Types/BehaviorTreeData.h"
#include "../HTWK_Debug/EnableLogs.h"
#include "../HTWK_Types/IntersectionState.h"

#define OID "htwk.us_obstacle_detection"
#define FILTER_NAME "HTWK Us Obstacle Detection Filter"

//defines for braking formula
#define BRAKE_SLOPE_SQUARE 29
#define BRAKE_SLOPE_LINEAR 8

#define MIN_US_DISTANCE_PROPERTY "min ultrasonic distance"
#define MIN_US_DISTANCE_DEFAULT 6
#define MIN_US_DISTANCE_DESCRIPTION "distance of ultrasonic sensors, at which the car will drive independet from obstacle recognition"
#define MIN_US_DISTANCE_MIN 3
#define MIN_US_DISTANCE_MAX 10

#define MIN_BRAKE_DISTANCE_PROPERTY "min braking distance"
#define MIN_BRAKE_DISTANCE_DEFAULT 9
#define MIN_BRAKE_DISTANCE_DESCRIPTION "minimal distance at which car stops"
#define MIN_BRAKE_DISTANCE_MIN 9
#define MIN_BRAKE_DISTANCE_MAX 300

#define MIN_STOP_TIME 500000

class UsObstacleDetection : public Leaf {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_Auxiliary);

private:
    tFloat64 donotBrakeDistancePropValue;
    int minBrakeDistancePropValue;
    tFloat32 currentSpeed;
    tUltrasonicStruct ultrasonicStruct;

    tTimeStamp timerForStop = 0;

    IntersectionState interState;

    tResult OnTrigger();

    tFloat64 BrakingDistance(const tFloat32 &speed);

    bool checkFront(const tUltrasonicStruct &ultrasonicStruct, const tFloat64 &checkValue);

    bool FrontFree(const tUltrasonicStruct &ultrasonicStruct, const tFloat32 &brakingDistance);

    bool checkBack(const tUltrasonicStruct &ultrasonicStruct, const tFloat64 &checkValue);

    bool BackFree(const tUltrasonicStruct &ultrasonicStruct, const tFloat32 &brakingDistance);


    bool DrivePossible(const tFloat32 &drivenSpeed,
                                       const tUltrasonicStruct &ultrasonicStruct);

public:
    UsObstacleDetection(const tChar *__info);

    virtual ~UsObstacleDetection();

    tResult Init(tInitStage eStage, __exception) override;
};

#endif //HTWK_US_OBSTACLE_DETECTION