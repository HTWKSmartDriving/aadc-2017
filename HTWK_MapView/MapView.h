#ifndef MAP_VIEW_HEADER
#define MAP_VIEW_HEADER

#include "stdafx.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <CarTrajectory.h>

#include "ADTF_OpenCV_helper.h"
#include "TrackMap.h"
#include "../HTWK_Types/IntersectionState.h"
#include "../HTWK_Behavior_Tree/HTWK_LeafNode/Leaf.h"
#include "../HTWK_Types/TrackingData.h"
#include "../HTWK_Types/PoliceData.h"
#include <MapElement.h>
#include <aadc_juryEnums.h>
#include <aadc_structs.h>
#include "../../HTWK_Debug/EnableLogs.h"
#define OID "htwk.mapview"
#define FILTER_NAME "HTWK BT:Node MapDriver"

#define STRAIGHT_IMAGE_PROPERTY "Straight Image"
#define TURN_SMALL_IMAGE_PROPERTY "Small Turn Image"
#define TURN_LARGE_IMAGE_PROPERTY "Large Turn Image"
#define LOT_IMAGE_PROPERTY "LotImg"
#define CROSSING_T_IMAGE_PROPERTY "T-Crossing Image"
#define CROSSING_PLUS_IMAGE_PROPERTY "Plus-Crossing Image"
#define S_TURN_PROPERTY_LEFT "S-Turn Left Image"
#define S_TURN_PROPERTY_RIGHT "S-Turn Right Image"

#define BOOL_PROPERTY "ChangeLane"
#define MAX_RADIUS_PROPERTY "maxRadius"

#define METERS_TO_PIXELS 100


#define DRIVE_ON_OTHER_LANE 5000000
#define STOP_AT_STOP_SIGN   3000000

#define MAXIMUM_RADIUS_FOR_TRACK_DRIVING 1.0

class MapView : public Leaf {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter)

private:

#ifdef DEBUG_VIDEO
    cv::Mat straightTile;
    cv::Mat turnSmallTile;
    cv::Mat turnLargeTile;
    cv::Mat tCrossingTile;
    cv::Mat plusCrossingTile;
    cv::Mat parkingLotTile;
    cv::Mat s_turn_left;
    cv::Mat s_turn_right;

    tBool tileImagesLoaded = tFalse;
#endif

private:
#ifdef DEBUG_VIDEO
    cVideoPin videoPin;
    tBitmapFormat outputBitmapFormat;
#endif

    cv::Mat mapImage;
//    cv::Mat gridMap = cv::Mat();


    TrackMap map;
    HTWKPoint currentPosition;
//    tFloat32 currentSpeed;
    tFloat32 currentOrientation;
    TilePoint *currentTilePoint;
    TilePoint *nextTilePoint;
    TilePoint *nextNextTilePoint;
    TilePoint *nextNextNextTilePoint;
    std::vector<tTrackingData> obstacleDataList;
    tPoliceData policeData; //Kuer!
    Orientation::TurnDirection nextTurn;

    std::vector<HTWKPoint> trajectoryPoints;

    IntersectionState interState;

    tBool isOffRoad = tFalse;

    stateCar state;

    tFloat32 maxRadius;

    bool mapInit;

    tTimeStamp laneChangeTimer;

    tBool pullOut = false;

#ifdef DEBUG_VIDEO
    int originCorrectionX = 0;
    int originCorrectionY = 0;
#endif

public:
    MapView(const tChar *__info);

    ~MapView();

    tResult Init(tInitStage eStage, __exception = nullptr) override;

private:
    tResult Shutdown(tInitStage eStage, IException **__exception_ptr) override;

    tResult OnTrigger() override;

    tResult calcTrackTrajectory();

    tResult calcTrackTrajectoryOppositeLane();

    tResult findSteering(cv::Mat &mat, const HTWKPoint &position, const tFloat32 &orientation);

    void setTurnSignal(IntersectionState state, Orientation::TurnDirection direction);

#ifdef DEBUG_VIDEO

    tResult CreateOutputPins(IException **__exception_ptr);

    tResult buildMapImage();

    tResult loadTileImage(cv::Mat &image, cFilename &path);

    tResult processVideo(cv::Mat &image);

    tResult markCurrentTilePosition(HTWKPoint position, tFloat32 orientation);

    void UpdateOutputImageFormat(const cv::Mat &outputImage);

    void insertTile(cv::Mat &mat, cv::Mat &tmpMat, const MapElement &element);

    cv::Point toCVPointInImage(const HTWKPoint &point);

    void printTileEntryPoints(cv::Mat &mat, MapElement &mapElement);

    void printTileExitPoints(cv::Mat &mat, MapElement &mapElement);

    tResult drawTrackGraph(cv::Mat &mat);

    tResult
    drawTrackGraphFromPosition(cv::Mat &image, const HTWKPoint &position, const float &Orientation,
                               const int &lookAheadDistance);


    tResult drawTrackTrajectory(cv::Mat &image, const std::vector<HTWKPoint> &trajectoryPoints);


    void drawCarTrajectory(const cv::Mat &mat, const HTWKPoint &position, const tFloat32 &orientation,
                           CarTrajectory &trajectoryCalculator, vector<HTWKPoint> &carTrajecotryPoints,
                           const pair<float, double> &bestTrajectory);


    void printTilePoint(cv::Mat &mat, TilePoint *pPoint);

    void printObstacles();

#endif
};

#endif // MAP_VIEW_HEADER
