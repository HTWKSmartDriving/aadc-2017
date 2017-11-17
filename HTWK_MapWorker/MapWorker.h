#ifndef MAP_WORKER_FILTER
#define MAP_WORKER_FILTER

#include "stdafx.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include "ADTF_OpenCV_helper.h"
#include "TrackMap.h"

#include <MapElement.h>
#include "../../HTWK_Debug/EnableLogs.h"
#include <MapTileManager.h>

#define OID "htwk.mapworker"
#define FILTER_NAME "HTWK MapWorker"

#define MAP_DATA_PROPERTY "Map Data File"
#define SIGN_DATA_PROPERTY "Sign Data File"

#define USE_XML_MAP_PROPERTY "Use XML Map"
#define USE_XML_SIGN_PROPERTY "Use XML Sign Data"


#define METERS_TO_PIXELS 100

#define INTERVAL_DEFAULT 100

class MapWorker : public adtf::cFilter {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter)

//Tile images
private:
    tBool useXMLMap;
    tBool useXMLSigns;

    cInputPin mapChangePin;

    cOutputPin mapPin;

    TrackMap map;
    MapTileManager tileManager;

    cObjectPtr<IMediaType> mapChangeMediaType;
    cObjectPtr<IMediaTypeDescription> mapChangeDescription;

public:
    MapWorker(const tChar *__info);

    virtual ~MapWorker();

    tResult Init(tInitStage eStage, __exception = nullptr) override;

private:
    tResult Shutdown(tInitStage eStage, IException **__exception_ptr) override;

    tResult CreateDescriptions(IException **__exception_ptr);

    tResult CreateInputPins(IException **__exception_ptr);

    tResult CreateOutputPins(IException **__exception_ptr);

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) override;

    tResult getMapFromFile();

    tResult transmitMapPtr(const IMediaSample *pMediaSample, TrackMap &map);

    tResult getSignsFromFile();
};

#endif // MAP_WORKER_FILTER
