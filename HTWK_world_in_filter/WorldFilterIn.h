#ifndef _WORLD_FILTER_HEADER_
#define _WORLD_FILTER_HEADER_

#include "stdafx.h"

#include <WorldService.h>
#include "../HTWK_Types/ManeuverList.h"
#include <tinyxml2.h>
#include <HTWKPoint.hpp>
#include <aadc_juryEnums.h>
#include "../HTWK_Map/TrackMap.h"
#include "../HTWK_Types/IntersectionState.h"
#include "../HTWK_Types/RoadSignDetectionResult.h"
#include "../HTWK_Types/ParkingLot.h"
#include "../HTWK_Types/ManueverEnum.h"
#include "../HTWK_Types/TrackingData.h"
#include "../HTWK_Types/PoliceData.h"
#include "../HTWK_Utils/VectorHelper.h"
#include <aadc_structs.h>
#include "../../HTWK_Debug/EnableLogs.h"
#include "../../HTWK_Utils/ImageHelper.h"
#include "../../HTWK_Debug/EnableLogs.h"

#define OID "htwk.world_filter_in"
#define FILTER_NAME "HTWK World In"

#define RECOGNIZE_SIGN_FOR_INTERSECTION_DISTANCE 2.0f

#define ROADSIGN_AND_PARKINGLOT_PROPERTY "Sign and Parkinglot XML"

#define DISTANCE_TO_BE_MOVED_CHECK_LOT "Distance moved to check changed parking lot"
#define DISTANCE_TO_BE_MOVED_VALUE 0.25f

class WorldFilterIn : public adtf::cFilter {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter)

private:
    cObjectPtr<WorldService> worldService;

    cInputPin maneuverListInput;
    cInputPin juryInput;
    cInputPin positionInput;
    cInputPin roadSigns;
    cInputPin mapInput;
    cInputPin leftLaneInput;
    cInputPin rightLaneInput;
    cInputPin centerInput;
    cInputPin obstacleInput;
    cInputPin usStructInput;

    cObjectPtr<IMediaType> maneuverListMediaType;
    cObjectPtr<IMediaTypeDescription> maneuverListDescription;

    cObjectPtr<IMediaType> juryMediaType;
    cObjectPtr<IMediaTypeDescription> juryDescription;

    cObjectPtr<IMediaType> signalMediaType;
    cObjectPtr<IMediaTypeDescription> signalDescription;

    cObjectPtr<IMediaType> positionMediaType;
    cObjectPtr<IMediaTypeDescription> positionDescription;

    cObjectPtr<IMediaType> usStructMediaType;
    cObjectPtr<IMediaTypeDescription> usStructDescription;

    cObjectPtr<IMediaType> roadSignExtMediaType;
    cObjectPtr<IMediaTypeDescription> roadSingExtDescription;

    std::string maneuverFileString;
    std::vector<tSector> sectorList;
    std::vector<tTrackingData> obstacleDataList;
    tPoliceData policeData; //Kuer
    TrackMap map;
    bool graphInitialized;
    tBool isOffRoad;
    tBool lostTrack;

    TilePoint *currentTilePoint;
    Orientation::TurnDirection nextTurn = Orientation::TurnDirection::STRAIGHT;

    IntersectionState interState;
    tRoadSignDetectionResult sign;
    tTimeStamp signTime;

    tInt16 maneuverEntry;
    tInt16 lastManeuverEntryInList;

    std::vector<ParkingLot> parkingLots;

    ManeuverEnum currentManeuver;
    juryActions actionId;

    cObjectPtr<IMediaTypeDescription> m_pDescParking;
    cOutputPin parkingUpdateOutputPin;
    struct parkingIDS {
        tBufferID m_szIDi16Identifier;
        tBufferID m_szIDF32X;
        tBufferID m_szIDF32Y;
        tBufferID m_szIDUi16Status;
        tBool IDsSet = false;
    } parkingIDS;
    HTWKPoint previousPosition;
    float checkForParkingLotsIfDistanceMoved = DISTANCE_TO_BE_MOVED_VALUE;
public:
    WorldFilterIn(const tChar *__info);

    virtual ~WorldFilterIn();

    tResult OnPinEvent(IPin *sourcePin, tInt eventCode, tInt param1, tInt param2, IMediaSample *mediaSample);

    tResult Init(tInitStage eStage, __exception = NULL);

private:
    tResult Start(ucom::IException **__exception_ptr);

    tResult Shutdown(tInitStage eStage, IException **__exception_ptr);

    tResult CreateDescriptions(IException **__exception_ptr);

    tResult CreateInputPins(IException **__exception_ptr);

    tResult LoadManeuverListAndPush();

    void switchIntersectionState(IntersectionState is);

    void updateTilePoints(TilePoint *&nextTilePoint, TilePoint *&nextNextTilePoint, TilePoint *&pPoint);

    bool nextCheckpoint(const HTWKPoint &position, const TilePoint *pPoint, const tFloat32 &heading);

    int getTilesTillIntersection(TilePoint *pPoint);

    void updateTurnParkingLotAndManeuver();

    tResult getParkinglotsFromFile();

    bool tilePointIsDeadEnd(TilePoint *pPoint);

    tResult sendParkingLotUpdate(const ParkingLot& lot);
};

#endif // _WORLD_FILTER_HEADER_