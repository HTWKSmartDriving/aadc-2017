#ifndef HTWK_CHANGE_SPEED
#define HTWK_CHANGE_SPEED

#include "stdafx.h"
#include "../HTWK_Behavior_Tree/HTWK_LeafNode/Leaf.h"
#include "../HTWK_Types/BehaviorTreeData.h"
#include "../HTWK_Debug/EnableLogs.h"
#include "../HTWK_Types/TrackingData.h"
#include "../HTWK_Utils/HTWKLine.h"
#include "../HTWK_Types/ZebraState.h"
#include "../HTWK_Types/IntersectionState.h"
#include "../HTWK_Utils/ImageHelper.h"
#include "../HTWK_Types/RoadSignEnum.h"


#define OID "htwk.change_speed"
#define FILTER_NAME "HTWK Change Speed Filter"

//speed properties
#define ROAD_SPEED_PROPERTY "roadSpeed"
#define INTERSECTION_SPEED_PROPERTY "intersectionSpeed"
#define NEARING_GIVE_WAY_SPEED_PROPERTY "nearingGiveWaySpeed"
#define NEARING_HAVE_WAY_SPEED_PROPERTY "nearingHaveWaySpeed"
#define NEARING_FULL_STOP_SPEED_PROPERTY "nearingFullStopSpeed"
#define NEARING_GENERIC_SPEED_PROPERTY "nearingGenericSpeed"

#define SPEED_ROAD 1.4f
#define SPEED_INTERSECTION 0.8f
#define SPEED_NEARING_GIVE_WAY 0.8f
#define SPEED_NEARING_HAVE_WAY 1.0f
#define SPEED_NEARING_FULL_STOP 0.5f
#define SPEED_NEARING_GENERIC 0.9f


#define DRIVE_ON_OTHER_LANE 5000000
#define STOP_AT_STOP_SIGN   3000000


#define LANE_WIDTH_PROPERTY "lane width"
#define LANE_WIDTH_DEFAULT 0.44

#define DETECT_DIST_PROPERTY "detect dist"
#define DETECT_DIST_DEFAULT 3.5
#define DETECT_DIST_DESCRIPTION "detetction distance in m"

#define SLOW_SPEED_PROPERTY "obstacle speed"
#define SLOW_SPEED_DEFAULT 0.3
#define SLOW_SPEED_DESCRIPTION "speed when driving in near of dangerous obstacles"

#define STOP_AT_PERSON_PROPERTY "stop at person"
#define STOP_AT_PERSON_DEFAULT 3.0
#define STOP_AT_PERSON_DESCRIPTION "stop in s"

#define ROAD_OBTACLE_DIST_PROPERTY "distance for obstacles on road"
#define ROAD_OBTACLE_DIST_DEFAULT 2

#define INTERSECTION_OBTACLE_DIST_PROPERTY "distance for obstacles on intersection"
#define INTERSECTION_OBTACLE_DIST_DEFAULT 3

#define PEDESTRIAN_OBTACLE_DIST_PROPERTY "distance for obstacles on pedestrian"
#define PEDESTRIAN_OBTACLE_DIST_DEFAULT 1





enum class ObstaclePos {
    LANE,
    OTHER_LANE,
    OFFROAD_R,
    OFFROAD_L

};

struct HTWKLane {
    HTWKPoint rightStart;
    HTWKPoint middleStart;
    HTWKPoint leftStart;
    HTWKPoint right;
    HTWKPoint middle;
    HTWKPoint left;
};

class ChangeSpeed : public Leaf {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_Auxiliary);

private:

    tResult OnTrigger();

    HTWKPoint currentPos;
    tFloat heading;
//    ZebraState zebraState;
    double leftLaneDist; // Abstand in x Richtung zur Lane (NICHT Weltkoordinaten!)
    double rightLaneDist;

    tFloat32 speedOnRoad;
    tFloat32 speedNearingHaveWay;
    tFloat32 speedNearingGiveWay;
    tFloat32 speedNearingFullStop;
    tFloat32 speedNearingGeneric;
    tFloat32 speedIntersection;
    tFloat32 currentSpeed;
    bool fullStopDone;
    stateCar state;
    tTimeStamp stopTimer;
    tTimeStamp pedestrianTimer = 0;
    int pedestrianTime;
    tTimeStamp stopPersonTimer = 0;
    int stopPersonTime;
    tFloat laneWidth;
    tFloat distRoad;
    tFloat distPedestrian;
    tFloat distIntersection;


    tBool pullOut = false;
    bool noObstacles = false;
    Orientation::TurnDirection nextTurn;
    ROAD_SIGN nextSign;

    bool firstObstacleSight = true;
    ObstaclePos firstObstaclePos;
    tFloat32 detectDist;
    tFloat32 pedestrianDetectDist = 0.4; //TODO Property!
    tFloat32 slowSpeed;

public:
    ChangeSpeed(const tChar *__info);

    virtual ~ChangeSpeed();

    tResult Init(tInitStage eStage, __exception) override;

protected:
    HTWKLane ExtractLanesFromPositionAndHeading(HTWKPoint carPos, tFloat carHeading);

    HTWKPoint PolarToCartesian(tFloat length, tFloat angle);

    ObstaclePos IsPositionOnRoad(cv::Point2f obstaclePosition);

//    tFloat VectorAngle(HTWKPoint vec1, HTWKPoint vec2);

    tResult FullBrake();

    tResult findUsualSpeed(const IntersectionState &interState);

    tResult findSpeedNew(const vector<tTrackingData> &obstacles, const IntersectionState &interState);

    tResult findObstacleSpeedOnRoad(const vector<tTrackingData> &obstacles);

    tResult findObstacleSpeedOnIntersection(const vector<tTrackingData> &obstacles);

    tResult findObstacleSpeedOnPedestrian(const vector<tTrackingData> &obstacles);

};

#endif //HTWK_CHANGE_SPEED