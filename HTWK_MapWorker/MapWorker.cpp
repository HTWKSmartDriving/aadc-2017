#include "MapWorker.h"
#include "../HTWK_MapPredictor/MapPredictor.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, MapWorker)

MapWorker::MapWorker(const tChar *__info) : cFilter(__info) {
    SetPropertyStr(MAP_DATA_PROPERTY, "map.xml");
    SetPropertyBool(MAP_DATA_PROPERTY
                            NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr(MAP_DATA_PROPERTY
                           NSSUBPROP_FILENAME
                           NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr(MAP_DATA_PROPERTY
                           NSSUBPROP_DESCRIPTION, "Map Data File to load");

    SetPropertyStr(SIGN_DATA_PROPERTY, "sign.xml");
    SetPropertyBool(SIGN_DATA_PROPERTY
                            NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr(SIGN_DATA_PROPERTY
                           NSSUBPROP_FILENAME
                           NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr(SIGN_DATA_PROPERTY
                           NSSUBPROP_DESCRIPTION, "Sign Data File to load");

    SetPropertyBool(USE_XML_MAP_PROPERTY, tTrue);
    SetPropertyBool(USE_XML_SIGN_PROPERTY, tTrue);
}

MapWorker::~MapWorker()
= default;

tResult MapWorker::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (StageFirst == eStage) {
        RETURN_IF_FAILED(CreateDescriptions(__exception_ptr));
        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
    } else if (StageNormal == eStage) {

    } else if (StageGraphReady == eStage) {
        useXMLMap = GetPropertyBool(USE_XML_MAP_PROPERTY);
        useXMLSigns = GetPropertyBool(USE_XML_SIGN_PROPERTY);

        //read the track map from xml file
        if (useXMLMap) {
            RETURN_IF_FAILED(getMapFromFile());
            map.setTileLinks();
        }

        if (useXMLSigns) {
            RETURN_IF_FAILED(getSignsFromFile());
        }

        tileManager.registerMapVector(map.asVector());
    }

    RETURN_NOERROR;
}

tResult MapWorker::Shutdown(tInitStage eStage, __exception) {
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult MapWorker::CreateDescriptions(IException **__exception_ptr) {
    cObjectPtr<IMediaDescriptionManager> descManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                         (tVoid **) &descManager, __exception_ptr));

    tChar const *mapChangeDescription = descManager->GetMediaDescription("tMapChange");
    RETURN_IF_POINTER_NULL(mapChangeDescription);
    mapChangeMediaType = new cMediaType(0, 0, 0, "tMapChange", mapChangeDescription,
                                        IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(
            mapChangeMediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &mapChangeDescription));

    RETURN_NOERROR;
}

tResult MapWorker::CreateInputPins(IException **__exception_ptr) {

    RETURN_IF_FAILED(mapChangePin.Create("map_change", new cMediaType(0, 0, 0, "tSignalValue"),
                                         static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&mapChangePin));

/*    RETURN_IF_FAILED(mapChangePin.Create("map_change", mapChangeMediaType,
                                         static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&mapChangePin));*/

    RETURN_NOERROR;
}


tResult MapWorker::CreateOutputPins(IException **__exception_ptr) {

    RETURN_IF_FAILED(
            mapPin.Create("map", new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED),
                          static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&mapPin));

    RETURN_NOERROR;
}

tResult MapWorker::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) {
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        // something was received
        RETURN_IF_POINTER_NULL(pMediaSample);

        if (pSource == &mapChangePin) {

/*            tInt16 x;
            tInt16 y;
            tInt16 Orientation;
            tInt8 id;

            {
                __adtf_sample_read_lock_mediadescription(mapChangeDescription, pMediaSample, inputCoder);
                inputCoder->Get("i16x", (tVoid *) &x);
                inputCoder->Get("i16y", (tVoid *) &y);
                inputCoder->Get("i16orientation", (tVoid *) &Orientation);
                inputCoder->Get("i8tileId", (tVoid *) &id);
            }

            unique_ptr<MapElement> me;

            switch (static_cast<MapTileType>(id)) {
                case MapTileType::STRAIGHT:
                    me.reset(new Straight(map.getIdIncrementor(), static_cast<MapTileType>(id), HTWKPoint(x, y),
                                          static_cast<Orientation::MapTile>(Orientation)));
                    break;
                case MapTileType::INTERSECTION_T:
                    me.reset(new TCrossing(map.getIdIncrementor(), static_cast<MapTileType>(id), HTWKPoint(x, y),
                                           static_cast<Orientation::MapTile>(Orientation)));
                    break;
                case MapTileType::INTERSECTION_PLUS:
                    me.reset(new PlusCrossing(map.getIdIncrementor(), static_cast<MapTileType>(id), HTWKPoint(x, y),
                                              static_cast<Orientation::MapTile>(Orientation)));
                    break;
                case MapTileType::TURN_LARGE:
                    me.reset(new LargeTurn(map.getIdIncrementor(), static_cast<MapTileType>(id), HTWKPoint(x, y),
                                           static_cast<Orientation::MapTile>(Orientation)));
                    break;
                case MapTileType::TURN_SMALL:
                    me.reset(new SmallTurn(map.getIdIncrementor(), static_cast<MapTileType>(id), HTWKPoint(x, y),
                                           static_cast<Orientation::MapTile>(Orientation)));
                    break;
                case MapTileType::LOT:
                    break;
                case MapTileType::TURN_S_LEFT:
                    break;
                case MapTileType::TURN_S_RIGHT:
                    break;
            }

            map.insertMapTile(*me);*/

            transmitMapPtr(pMediaSample, map);
        }
    }

    RETURN_NOERROR;
}

tResult MapWorker::getMapFromFile() {
    // Get filename of map file
    cFilename fileMap = GetPropertyStr(MAP_DATA_PROPERTY);
    map.setFile(fileMap);
    RETURN_IF_FAILED(map.readXMLMap());

    RETURN_NOERROR;
}

tResult MapWorker::getSignsFromFile() {
    cFilename fileSigns = GetPropertyStr(SIGN_DATA_PROPERTY);

    ADTF_GET_CONFIG_FILENAME(fileSigns);
    fileSigns = fileSigns.CreateAbsolutePath(".");

    MapPredictor predictor = MapPredictor(fileSigns);
    std::map<int, std::map<int, MapElement>> tilesMap = predictor.getPredictedMap();

    for (int y = predictor.getBounds().maxY; y >= predictor.getBounds().minY; y--) {
        for (int x = predictor.getBounds().minX; x <= predictor.getBounds().maxX; x++) {
            if (tilesMap[x][y].isSet()) {
                map.insertMapTile(tilesMap[x][y]);
            }
        }
    }

    RETURN_NOERROR;
}

tResult MapWorker::transmitMapPtr(const IMediaSample *pMediaSample, TrackMap &map) {
    if (map.empty()) {
        RETURN_NOERROR;
    }

    std::vector<MapElement> mapTiles = map.asVector();

    cObjectPtr<IMediaSample> pMapSample;
    if (IS_OK(AllocMediaSample(&pMapSample))) {
        pMapSample->Update(pMediaSample->GetTime(), &mapTiles.front(), sizeof(MapElement) * mapTiles.size(), 0);
        RETURN_IF_FAILED(mapPin.Transmit(pMapSample));
    }
    RETURN_NOERROR;
}

