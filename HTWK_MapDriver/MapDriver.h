#ifndef _SIGNAL_GENERATOR_HEADER_
#define _SIGNAL_GENERATOR_HEADER_

#include "stdafx.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <CarTrajectory.h>
#include "../../HTWK_Debug/EnableLogs.h"
#include "ADTF_OpenCV_helper.h"
#include "TrackMap.h"

#include <MapElement.h>

#define OID "htwk.mapdriver"
#define FILTER_NAME "HTWK MapDriver"

#define INTERVAL_PROPERTY "Interval"
#define MAP_DATA_PROPERTY "Map Data File"
#define STRAIGHT_IMAGE_PROPERTY "Straight Image"
#define TURN_SMALL_IMAGE_PROPERTY "Small Turn Image"
#define TURN_LARGE_IMAGE_PROPERTY "Large Turn Image"
#define LOT_IMAGE_PROPERTY "LotImg"
#define CROSSING_T_IMAGE_PROPERTY "T-Crossing Image"
#define CROSSING_PLUS_IMAGE_PROPERTY "Plus-Crossing Image"
#define NEXT_TURN_PROPERTY "Next Turn"

#define METERS_TO_PIXELS 100

#define INTERVAL_DEFAULT 100

class MapDriver : public adtf::cTimeTriggeredFilter {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter)

    tResult Cycle(IException **__exception_ptr) override;

//Tile images
private:

    tBool graphInitialized;

private:
    cOutputPin steeringPin;
    cInputPin positionPin;

    cObjectPtr<IMediaType> signalValueStruct;
    cObjectPtr<IMediaTypeDescription> signalDescription;
    tBool m_bIDsSignalSet;

    cObjectPtr<IMediaType> positionStruct;
    cObjectPtr<IMediaTypeDescription> positionDescription;
    tBool m_bIDsPositionSet;

    TrackMap map;

    HTWKPoint currentPosition;
    tFloat32 currentOrientation;

    int nextTurn;

    TilePoint *currentTilePoint;

    std::vector<HTWKPoint> trajectoryPoints;

    /*! the id for the f32value of the media description for the signal value input pins */
    tBufferID m_szIDSignalF32Value;
    /*! the id for the arduino time stamp of the media description for the signal value input pins */
    tBufferID m_szIDSignalArduinoTimestamp;

    tBufferID idPositionX;
    tBufferID idPositionY;
    tBufferID idRadius;
    tBufferID idSpeed;
    tBufferID idHeading;


public:
    MapDriver(const tChar *__info);

    virtual ~MapDriver();

    tResult Init(tInitStage eStage, __exception = nullptr) override;

private:
    tResult Shutdown(tInitStage eStage, IException **__exception_ptr) override;

    tResult CreateDescriptions(IException **__exception_ptr);

    tResult CreateInputPins(IException **__exception_ptr);

    tResult CreateOutputPins(IException **__exception_ptr);

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) override;

    tResult getMapFromFile();

    tResult TransmitValue(cOutputPin &pin, const tFloat32 value);

    tResult calcTrackTrajectory();

    tResult findSteeringAndTransmit(const HTWKPoint &position, const tFloat32 &orientation);
};

#endif // _SIGNAL_GENERATOR_HEADER_
