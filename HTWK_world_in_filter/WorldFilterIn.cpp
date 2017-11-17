#include "WorldFilterIn.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, WorldFilterIn)

WorldFilterIn::WorldFilterIn(const tChar *__info) : cFilter(__info) {
    SetPropertyStr(ROADSIGN_AND_PARKINGLOT_PROPERTY, "roadSigns.xml");
    SetPropertyBool(ROADSIGN_AND_PARKINGLOT_PROPERTY
                            NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr(ROADSIGN_AND_PARKINGLOT_PROPERTY
                           NSSUBPROP_FILENAME
                           NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr(ROADSIGN_AND_PARKINGLOT_PROPERTY
                           NSSUBPROP_DESCRIPTION, "Roadsign and Parkinglot File to load");

    SetPropertyFloat(DISTANCE_TO_BE_MOVED_CHECK_LOT, DISTANCE_TO_BE_MOVED_VALUE);
    SetPropertyStr(DISTANCE_TO_BE_MOVED_CHECK_LOT NSSUBPROP_DESCRIPTION,
                   "Minimum moved Distance in Metern for new ParkingLotCheck");
    SetPropertyFloat(DISTANCE_TO_BE_MOVED_CHECK_LOT NSSUBPROP_MIN, 0.0);
    SetPropertyFloat(DISTANCE_TO_BE_MOVED_CHECK_LOT NSSUBPROP_MAX, 2.0);

    graphInitialized = tFalse;
    isOffRoad = tFalse;
    lostTrack = tFalse;

    interState = IntersectionState::ON_ROAD;
    currentManeuver = ManeuverEnum::STRAIGHT;
}

WorldFilterIn::~WorldFilterIn() {
}

tResult WorldFilterIn::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (StageFirst == eStage) {
        RETURN_IF_FAILED(CreateDescriptions(__exception_ptr));
        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));

        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid **) &pDescManager, __exception_ptr));

        tChar const *strDesc = pDescManager->GetMediaDescription("tParkingSpace");
        RETURN_IF_POINTER_NULL(strDesc);
        cObjectPtr<IMediaType> pTypeParking = new cMediaType(0, 0, 0, "tParkingSpace", strDesc,
                                                              IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        // create the position OutputPin
        RETURN_IF_FAILED(parkingUpdateOutputPin.Create("ParkingJuryUpdate", pTypeParking, this));
        RETURN_IF_FAILED(RegisterPin(&parkingUpdateOutputPin));
        // set the description for the extended marker pin
        RETURN_IF_FAILED(pTypeParking->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &m_pDescParking));

    } else if (StageNormal == eStage) {

    } else if (StageGraphReady == eStage) {
        RETURN_IF_FAILED(
                _runtime->GetObject(OID_WORLD_SERVICE, IID_WORLD_INTERFACE, (tVoid **) &worldService, __exception_ptr));
        worldService->Clear();

        RETURN_IF_FAILED(getParkinglotsFromFile());
    }

    RETURN_NOERROR;
}

tResult WorldFilterIn::Shutdown(tInitStage eStage, __exception) {
    if (worldService != NULL) {
        worldService->Clear();
        worldService = NULL;
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult
WorldFilterIn::OnPinEvent(IPin *sourcePin, tInt eventCode, tInt param1, tInt param2, IMediaSample *mediaSample) {
    if (eventCode != IPinEventSink::PE_MediaSampleReceived) {
        RETURN_NOERROR;
    }

    worldService->Pull(CAR_MANEUVER_ENTRY, maneuverEntry);
    updateTurnParkingLotAndManeuver();

    RETURN_IF_POINTER_NULL(mediaSample);

    if (sourcePin == &maneuverListInput) {
        {
            // focus for sample read lock
            __adtf_sample_read_lock_mediadescription(maneuverListDescription, mediaSample, pCoder);

            std::vector<tSize> vecDynamicIDs;

            // retrieve number of elements by providing NULL as first paramter
            tSize szBufferSize = 0;
            if (IS_OK(pCoder->GetDynamicBufferIDs(NULL, szBufferSize))) {
                // create a buffer depending on the size element
                std::string pcBuffer;
                pcBuffer.resize(szBufferSize);
                vecDynamicIDs.resize(szBufferSize);

                // get the dynamic ids (we already got the first "static" size element)
                if (IS_OK(pCoder->GetDynamicBufferIDs(&(vecDynamicIDs.front()), szBufferSize))) {
                    // iterate over all elements
                    for (tUInt32 nIdx = 0; nIdx < vecDynamicIDs.size(); ++nIdx) {
                        // get the value and put it into the buffer
                        pCoder->Get(vecDynamicIDs[nIdx], &pcBuffer[nIdx]);
                    }

                    // set the resulting char buffer to the string object
                    maneuverFileString = pcBuffer;
                }
            }
        }

        LoadManeuverListAndPush();

        worldService->Push(WORLD_PARKING_LOTS, parkingLots);
    } else if (sourcePin == &positionInput) {
        updateTurnParkingLotAndManeuver();

        tFloat32 pos_x;
        tFloat32 pos_y;
        tFloat32 radius;
        tFloat32 speed;
        tFloat32 heading;

        {
            __adtf_sample_read_lock_mediadescription(positionDescription, mediaSample, inputCoder);
            inputCoder->Get("f32x", (tVoid *) &pos_x);
            inputCoder->Get("f32y", (tVoid *) &pos_y);
            inputCoder->Get("f32radius", (tVoid *) &radius);
            inputCoder->Get("f32speed", (tVoid *) &speed);
            inputCoder->Get("f32heading", (tVoid *) &heading);
        }

        HTWKPoint position(pos_x, pos_y);
        if(position.dist(previousPosition) > checkForParkingLotsIfDistanceMoved){
            std::vector<ParkingLot> checkParkingLots;
            worldService->Pull(WORLD_PARKING_LOTS, checkParkingLots);
            for(auto& oldLots : parkingLots){
                for(const auto& newLots : checkParkingLots){
                    if(oldLots.id == newLots.id){
                        if(oldLots.status != newLots.status){
                            sendParkingLotUpdate(newLots);
                            oldLots.status = newLots.status;
                        }
                        break;
                    }
                }
            }
            previousPosition = position;
        }

#ifdef DEBUG_WORLD_FILTER_IN_LOG
        std::cout << "Position: x: " << pos_x << " y: " << pos_y << std::endl;
#endif
        worldService->Push(WORLD_CURRENT_POSITION, position);
        worldService->Push(WORLD_CURRENT_RADIUS, radius);
        worldService->Push(WORLD_CURRENT_SPEED, speed);
        worldService->Push(WORLD_CURRENT_HEADING, heading);

        TilePoint *nextTilePoint = nullptr;
        TilePoint *nextNextTilePoint = nullptr;
        TilePoint *nextNextNextTilePoint = nullptr;

        //calculate lane point
        if (!graphInitialized || isOffRoad || actionId == juryActions::action_GETREADY) {
            if (!map.empty()) {
                //LOG_INFO("INIT");
                currentTilePoint = map.getTilePointByPosition(position, heading);

                if(nullptr != currentTilePoint) {
                    if(lostTrack) {
                        isOffRoad = tFalse;
                        lostTrack = tFalse;
                    }
                } else {
                    isOffRoad = tTrue;
                    lostTrack = tTrue;
                }

                graphInitialized = tTrue;

                updateTilePoints(nextTilePoint, nextNextTilePoint, nextNextNextTilePoint);
                worldService->Push(WORLD_CURRENT_TILE_POINT, currentTilePoint);
            }
        } else {
            updateTilePoints(nextTilePoint, nextNextTilePoint, nextNextNextTilePoint);

            if (nextCheckpoint(position, nextTilePoint, heading)) {
                currentTilePoint = nextTilePoint;
                updateTilePoints(nextTilePoint, nextNextTilePoint, nextNextNextTilePoint);
            }

            if(tilePointIsDeadEnd(nextTilePoint)) {
                isOffRoad = tTrue;
            }

            int tilesTillIntersection = getTilesTillIntersection(currentTilePoint);

            if (tilesTillIntersection == 0) {
                switchIntersectionState(IntersectionState::ON_INTERSECTION);
            } else if (tilesTillIntersection <= 3) {
                if (HTWKMathUtils::get3DVectorNorm(sign.tVec.x, sign.tVec.y, sign.tVec.z) <
                    RECOGNIZE_SIGN_FOR_INTERSECTION_DISTANCE) {
                    switch (sign.id) {
                        case ROAD_SIGN::UNMARKED:
                            switchIntersectionState(IntersectionState::NEARING_GIVE_WAY);
                            break;
                        case ROAD_SIGN::STOP:
                            switchIntersectionState(IntersectionState::NEARING_FULL_STOP);
                            break;
                        case ROAD_SIGN::HAVE_WAY:
                            switchIntersectionState(IntersectionState::NEARING_HAVE_WAY);
                            break;
                        case ROAD_SIGN::GIVE_WAY:
                            switchIntersectionState(IntersectionState::NEARING_GIVE_WAY);
                            break;
                        default:
                            switchIntersectionState(IntersectionState::NEARING_GENERIC);
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                        LOG_ERROR("Wrong or no sign detected for next intersection");
                        LOG_INFO("Switching to GENERIC intersection handling");
#endif
                            break;
                    }
                } else {
                    switchIntersectionState(IntersectionState::NEARING_GENERIC);
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_ERROR("No sign detected for next intersection");
                    LOG_INFO("Switching to GENERIC intersection handling");
#endif
                }

                if ((interState == IntersectionState::NEARING_FULL_STOP ||
                     interState == IntersectionState::NEARING_GIVE_WAY) &&
                    tilesTillIntersection == 1 &&
                    HTWKUtils::Equals(position, *nextTilePoint, 0.7)) {
                    switchIntersectionState(IntersectionState::STOP_NOW);
                }
            } else {
                switchIntersectionState(IntersectionState::ON_ROAD);
            }

            worldService->Push(WORLD_CURRENT_TILE_POINT, currentTilePoint);
            worldService->Push(WORLD_NEXT_TILE_POINT, nextTilePoint);
            worldService->Push(WORLD_NEXT_NEXT_TILE_POINT, nextNextTilePoint);
            worldService->Push(WORLD_NEXT_NEXT_NEXT_TILE_POINT, nextNextNextTilePoint);

        }

        worldService->Push(WORLD_INTERSECTION_STATE, interState);
        worldService->Push(WORLD_IS_OFFROAD, isOffRoad);

    } else if (sourcePin == &juryInput) {
        tInt8 actionInt;

        {
            __adtf_sample_read_lock_mediadescription(juryDescription, mediaSample, inputCoder);
            inputCoder->Get("i8ActionID", (tVoid *) &actionInt);
            inputCoder->Get("i16ManeuverEntry", (tVoid *) &maneuverEntry);
        }

        actionId = static_cast<juryActions>(actionInt);

        worldService->Push(WORLD_ACTION_ID, actionId);

        updateTurnParkingLotAndManeuver();
    } else if (sourcePin == &roadSigns) {

        tRoadSignExt data;
        {
            __adtf_sample_read_lock_mediadescription(roadSingExtDescription, mediaSample, inputCoder);
            inputCoder->Get("i16Identifier", (tVoid *) &data.i16Identifier);
            inputCoder->Get("f32Imagesize", (tVoid *) &data.f32Imagesize);
            inputCoder->Get("af32TVec", (tVoid *) &data.af32TVec);
            inputCoder->Get("af32RVec", (tVoid *) &data.af32RVec);
        }

        sign.id = static_cast<ROAD_SIGN>(data.i16Identifier);
        sign.tVec = cv::Point3f(data.af32TVec[0], data.af32TVec[1],data.af32TVec[2]);
        sign.rVec = cv::Point3f(data.af32RVec[0],data.af32RVec[1],data.af32RVec[2]);
        sign.imagesize = data.f32Imagesize;

        signTime = _clock->GetStreamTime(); //signTime,signTime,signTime -> Nice Rhyme :)

        worldService->Push(WORLD_ROAD_SIGN_EXT, sign);
        worldService->Push(WORLD_LAST_ROAD_SIGN, signTime);

    } else if (sourcePin == &mapInput) {
        if (!graphInitialized) {
            map.clear();

            {
                __adtf_sample_read_lock(mediaSample, MapElement, pData);

                for (unsigned int i = 0; i < (mediaSample->GetSize() / sizeof(MapElement)); i++) {
                    map.push(pData[i]);
                }
            }

            std::vector<MapElement> mapElementsVector = map.asVector();

            worldService->Push(WORLD_MAP, mapElementsVector);
        }
    } else if (sourcePin == &leftLaneInput) {
        tFloat32 leftLane;

        {
            __adtf_sample_read_lock_mediadescription(signalDescription, mediaSample, inputCoder);
            inputCoder->Get("f32Value", (tVoid *) &leftLane);
        }

        worldService->Push(WORLD_LANE_LEFT, leftLane);
    } else if (sourcePin == &rightLaneInput) {
        tFloat32 rightLane;

        {
            __adtf_sample_read_lock_mediadescription(signalDescription, mediaSample, inputCoder);
            inputCoder->Get("f32Value", (tVoid *) &rightLane);
        }

        worldService->Push(WORLD_LANE_RIGHT, rightLane);
    } else if (sourcePin == &centerInput) {
        tFloat32 picCenter;

        {
            __adtf_sample_read_lock_mediadescription(signalDescription, mediaSample, inputCoder);
            inputCoder->Get("f32Value", (tVoid *) &picCenter);
        }

        worldService->Push(WORLD_LANE_CENTER, picCenter);
    } else if (sourcePin == &obstacleInput) {
        {
            __adtf_sample_read_lock(mediaSample, tTrackingData, pData);
            htwk::recieveVector(mediaSample, obstacleDataList);
        }
        if(obstacleDataList.front().status == ObstacleStatus::REARCAM){
            policeData.countSeen++;
            policeData.lastSeen = _clock->GetTime();
            worldService->Push(THIS_IS_THE_POLICE, policeData);
        } else{
            worldService->Push(WORLD_OBSTACLES, obstacleDataList);
        }
    } else if (sourcePin == &usStructInput) {
        tUltrasonicStruct ultrasonicStruct{};

        {
            // this will aquire the read lock on the sample and declare and initialize a pointer to the data
            __sample_read_lock(mediaSample, tUltrasonicStruct, pData);
            // now we can access the sample data through the pointer
            ultrasonicStruct = *pData;
        }
        worldService->Push(WORLD_ULTRASONICS, ultrasonicStruct);
    }
    RETURN_NOERROR;
}

void WorldFilterIn::updateTurnParkingLotAndManeuver() {
    worldService->Push(CAR_MANEUVER_ENTRY, maneuverEntry);

    int targetParkingLot = -1;
    ManeuverEnum nextManeuver = ManeuverEnum::STRAIGHT;

    if (maneuverEntry <= lastManeuverEntryInList) {
        for (const auto &sector: sectorList) {
            for (const auto &maneuver: sector.maneuverList) {
                if (maneuver.id == maneuverEntry) {
                    cStringList splitedManeuver;
                    maneuver.action.Split(splitedManeuver, " ");

                    if (strcmp(splitedManeuver[0], "straight") == 0) { //this means match at position 0
                        nextTurn = Orientation::TurnDirection::STRAIGHT;
                        nextManeuver = ManeuverEnum::STRAIGHT;
                    } else if (strcmp(splitedManeuver[0], "left") == 0) {
                        nextTurn = Orientation::TurnDirection::LEFT;
                        nextManeuver = ManeuverEnum::LEFT;
                    } else if (strcmp(splitedManeuver[0], "right") == 0) {
                        nextTurn = Orientation::TurnDirection::RIGHT;
                        nextManeuver = ManeuverEnum::RIGHT;
                    } else if (strcmp(splitedManeuver[0], "cross_parking") == 0) {
                        targetParkingLot = splitedManeuver.Get(1).AsInt();

                        nextManeuver = ManeuverEnum::CROSS_PARKING;
                        nextTurn = Orientation::TurnDirection::STRAIGHT;
                    } else if (strcmp(splitedManeuver[0], "pull_out_right") == 0) {
                        nextTurn = Orientation::TurnDirection::STRAIGHT;
                        nextManeuver = ManeuverEnum::PULLOUT_RIGHT;
                    } else if (strcmp(splitedManeuver[0], "pull_out_left") == 0) {
                        nextTurn = Orientation::TurnDirection::STRAIGHT;
                        nextManeuver = ManeuverEnum::PULLOUT_LEFT;
                    } else {
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                        LOG_ERROR("Maneuver not found");
#endif
                    }

                    worldService->Push(WORLD_TARGET_PARKING_LOT, targetParkingLot);
                    worldService->Push(WORLD_NEXT_TURN, nextTurn);

                    if (nextManeuver != currentManeuver) {
                        if (currentManeuver == ManeuverEnum::PULLOUT_LEFT) {
                            graphInitialized = tFalse;
                            interState = IntersectionState::ON_ROAD;
                        }

                        worldService->Push(WORLD_NEXT_MANEUVER, nextManeuver);
                        currentManeuver = nextManeuver;
                    }

                    return;
                }
            }
        }
    }
}

void WorldFilterIn::updateTilePoints(TilePoint *&nextTilePoint, TilePoint *&nextNextTilePoint,
                                     TilePoint *&nextNextNextTilePoint) { //

    if (currentTilePoint == nullptr || tilePointIsDeadEnd(currentTilePoint)) {
        isOffRoad = tTrue;
        return;
    }

    if (currentTilePoint->next[static_cast<int>(nextTurn)] != nullptr) {
        nextTilePoint = currentTilePoint->next[static_cast<int>(nextTurn)];

        if (nextTilePoint->next[static_cast<int>(nextTurn)] != nullptr) {
            nextNextTilePoint = nextTilePoint->next[static_cast<int>(nextTurn)];

            if (nextNextTilePoint->next[static_cast<int>(nextTurn)] != nullptr) {
                nextNextNextTilePoint = nextNextTilePoint->next[static_cast<int>(nextTurn)];
            } else {
                if (nextNextTilePoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)] != nullptr) {
                    nextNextNextTilePoint = nextNextTilePoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)];
                } else {
                    nextNextNextTilePoint = nullptr;
                    return;
                }
            }
        } else {
            if (nextTilePoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)] != nullptr) {
                nextNextTilePoint = nextTilePoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)];
            } else {
                nextNextTilePoint = nullptr;
                nextNextNextTilePoint = nullptr;
                return;
            }
            if (nextNextTilePoint->next[static_cast<int>(nextTurn)] != nullptr) {
                nextNextNextTilePoint = nextNextTilePoint->next[static_cast<int>(nextTurn)];
            } else {
                if (nextNextTilePoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)] != nullptr) {
                    nextNextNextTilePoint = nextNextTilePoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)];
                } else {
                    nextNextNextTilePoint = nullptr;
                    return;
                }
            }
        }
    } else {
        if (currentTilePoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)] != nullptr) {
            nextTilePoint = currentTilePoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)];
        } else {
            nextTilePoint = nullptr;
            nextNextTilePoint = nullptr;
            nextNextNextTilePoint = nullptr;
            return;
        }

        if (nextTilePoint->next[static_cast<int>(nextTurn)] != nullptr) {
            nextNextTilePoint = nextTilePoint->next[static_cast<int>(nextTurn)];

            if (nextNextTilePoint->next[static_cast<int>(nextTurn)] != nullptr) {
                nextNextNextTilePoint = nextNextTilePoint->next[static_cast<int>(nextTurn)];
            } else {
                if (nextNextTilePoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)] != nullptr) {
                    nextNextNextTilePoint = nextNextTilePoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)];
                } else {
                    nextNextNextTilePoint = nullptr;
                    return;
                }
            }
        } else {
            if (nextTilePoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)] != nullptr) {
                nextNextTilePoint = nextTilePoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)];
            } else {
                nextNextTilePoint = nullptr;
                nextNextNextTilePoint = nullptr;
                return;
            }

            if (nextNextTilePoint->next[static_cast<int>(nextTurn)] != nullptr) {
                nextNextNextTilePoint = nextNextTilePoint->next[static_cast<int>(nextTurn)];
            } else {
                if (nextNextTilePoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)] != nullptr) {
                    nextNextNextTilePoint = nextNextTilePoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)];
                } else {
                    nextNextNextTilePoint = nullptr;
                    return;
                }
            }
        }
    }
}

tResult WorldFilterIn::CreateDescriptions(IException **__exception_ptr) {
    cObjectPtr<IMediaDescriptionManager> descManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                         (tVoid **) &descManager, __exception_ptr));

    // get enum
    tChar const *enumManeuverListDescription = descManager->GetMediaDescription("tManeuverList");
    RETURN_IF_POINTER_NULL(enumManeuverListDescription);
    maneuverListMediaType = new cMediaType(0, 0, 0, "tManeuverList", enumManeuverListDescription,
                                           IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(
            maneuverListMediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &maneuverListDescription));

    // get tSignalValue
    tChar const *signalValueDescription = descManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(signalValueDescription);
    signalMediaType = new cMediaType(0, 0, 0, "tSignalValue", signalValueDescription,
                                     IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(signalMediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &signalDescription));

    // get tJuryStruct
    tChar const *juryStructDescription = descManager->GetMediaDescription("tJuryStruct");
    RETURN_IF_POINTER_NULL(juryStructDescription);
    juryMediaType = new cMediaType(0, 0, 0, "tJuryStruct", juryStructDescription,
                                   IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(juryMediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &juryDescription));

    // get tPosition
    tChar const *posDescription = descManager->GetMediaDescription("tPosition");
    RETURN_IF_POINTER_NULL(posDescription);
    positionMediaType = new cMediaType(0, 0, 0, "tPosition", posDescription,
                                       IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(positionMediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &positionDescription));

    // get tRoadSignExt
    tChar const *roadsignextDescription = descManager->GetMediaDescription("tRoadSignExt");
    RETURN_IF_POINTER_NULL(roadsignextDescription);
    roadSignExtMediaType = new cMediaType(0, 0, 0, "tRoadSignExt", roadsignextDescription,
                                          IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(
            roadSignExtMediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &roadSingExtDescription));

    // get tUsStruct
    tChar const *usDescription = descManager->GetMediaDescription("tUltrasonicStruct");
    RETURN_IF_POINTER_NULL(usDescription);
    usStructMediaType = new cMediaType(0, 0, 0, "tUltrasonicStruct", usDescription,
                                       IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(
            usStructMediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &usStructDescription));

    RETURN_NOERROR;
}

tResult WorldFilterIn::CreateInputPins(IException **__exception_ptr) {
    RETURN_IF_FAILED(
            maneuverListInput.Create("ManeuverList", maneuverListMediaType, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&maneuverListInput));

    RETURN_IF_FAILED(positionInput.Create("Position", positionMediaType, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&positionInput));

    RETURN_IF_FAILED(juryInput.Create("JuryStruct", juryMediaType, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&juryInput));

    RETURN_IF_FAILED(
            roadSigns.Create("RoadSignExt", roadSignExtMediaType, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&roadSigns));

    cObjectPtr<IMediaType> pInputType;

    RETURN_IF_FAILED(
            leftLaneInput.Create("LeftLane", signalMediaType, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&leftLaneInput));

    RETURN_IF_FAILED(
            rightLaneInput.Create("RightLane", signalMediaType, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&rightLaneInput));

    RETURN_IF_FAILED(
            centerInput.Create("PictureCenter", signalMediaType, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&centerInput));

    RETURN_IF_FAILED(
            usStructInput.Create("UsStruct", usStructMediaType, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&usStructInput));

    RETURN_IF_FAILED(
            mapInput.Create("map", new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED),
                            static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&mapInput));

    RETURN_IF_FAILED(
            obstacleInput.Create("obstacles",
                                 new adtf::cMediaType(MEDIA_TYPE_TRACKING_DATA, MEDIA_SUBTYPE_TRACKING_DATA),
                                 static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&obstacleInput));

    RETURN_NOERROR;
}


tResult WorldFilterIn::LoadManeuverListAndPush() {
    sectorList.clear();

    tinyxml2::XMLDocument doc;
    doc.Parse(maneuverFileString.c_str());

    tinyxml2::XMLNode *pRoot = doc.FirstChild();

    for (tinyxml2::XMLElement *xmlSector = pRoot->NextSibling()->FirstChildElement();
         xmlSector != nullptr; xmlSector = xmlSector->NextSiblingElement()) {
        tinyxml2::XMLElement *tmpXmlSector = xmlSector;

        tSector sector;
        sector.id = tmpXmlSector->IntAttribute("id");

        for (tinyxml2::XMLElement *xmlMan = tmpXmlSector->FirstChildElement();
             xmlMan != nullptr; xmlMan = xmlMan->NextSiblingElement()) {
            tinyxml2::XMLElement *tmpXmlMan = xmlMan;

            tAADC_Maneuver man;
            man.id = tmpXmlMan->IntAttribute("id");
            man.action = tmpXmlMan->Attribute("action");
            sector.maneuverList.push_back(man);
        }

        sectorList.push_back(sector);
    }

    lastManeuverEntryInList = sectorList.back().maneuverList.back().id;

    worldService->Push(WORLD_LAST_MANEUVER_ENTRY_IN_LIST, lastManeuverEntryInList);

    RETURN_NOERROR;
}

void WorldFilterIn::switchIntersectionState(IntersectionState is) {
    switch (interState) {
        case IntersectionState::ON_ROAD:
            switch (is) {
                case IntersectionState::ON_ROAD:
                    break;
                case IntersectionState::NEARING_HAVE_WAY:
                    interState = is;
                    break;
                case IntersectionState::NEARING_GIVE_WAY:
                    interState = is;
                    break;
                case IntersectionState::NEARING_FULL_STOP:
                    interState = is;
                    break;
                case IntersectionState::STOP_NOW:
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_ERROR("Cant switch from ON_ROAD to STOP_NOW ");
#endif
                    break;
                case IntersectionState::ON_INTERSECTION:
                    interState = is;
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_INFO("Switch from ON_ROAD to ON_INTERSECTION without NEARING handling");
#endif
                    break;
                case IntersectionState::NEARING_GENERIC:
                    interState = is;
                    break;
            }
            break;
        case IntersectionState::NEARING_HAVE_WAY:
            switch (is) {
                case IntersectionState::ON_ROAD:
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_ERROR("Cant switch from NEARING_HAVE_WAY to ON_ROAD ");
#endif
                    break;
                case IntersectionState::NEARING_HAVE_WAY:
                    break;
                case IntersectionState::NEARING_GIVE_WAY:
                    interState = is;
                    break;
                case IntersectionState::NEARING_FULL_STOP:
                    interState = is;
                    break;
                case IntersectionState::STOP_NOW:
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_ERROR("Cant switch from NEARING_HAVE_WAY to STOP_NOW ");
#endif
                    break;
                case IntersectionState::ON_INTERSECTION:
                    interState = is;
                    break;
                case IntersectionState::NEARING_GENERIC:
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_ERROR("Cant switch from NEARING_HAVE_WAY to NEARING_GENERIC ");
#endif
                    break;
            }
            break;
        case IntersectionState::NEARING_GIVE_WAY:
            switch (is) {
                case IntersectionState::ON_ROAD:
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_ERROR("Cant switch from NEARING_GIVE_WAY to ON_ROAD ");
#endif
                    break;
                case IntersectionState::NEARING_HAVE_WAY:
                    interState = is;
                    break;
                case IntersectionState::NEARING_GIVE_WAY:
                    break;
                case IntersectionState::NEARING_FULL_STOP:
                    interState = is;
                    break;
                case IntersectionState::STOP_NOW:
                    interState = is;
                    break;
                case IntersectionState::ON_INTERSECTION:
                    interState = is;
                    break;
                case IntersectionState::NEARING_GENERIC:
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_ERROR("Cant switch from NEARING_GIVE_WAY to NEARING_GENERIC ");
#endif
                    break;
            }
            break;
        case IntersectionState::NEARING_FULL_STOP:
            switch (is) {
                case IntersectionState::ON_ROAD:
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_ERROR("Cant switch from NEARING_FULL_STOP to ON_ROAD ");
#endif
                    break;
                case IntersectionState::NEARING_HAVE_WAY:
                    interState = is;
                    break;
                case IntersectionState::NEARING_GIVE_WAY:
                    interState = is;
                    break;
                case IntersectionState::NEARING_FULL_STOP:
                    break;
                case IntersectionState::STOP_NOW:
                    interState = is;
                    break;
                case IntersectionState::ON_INTERSECTION:
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_ERROR("Cant switch from NEARING_FULL_STOP to ON_INTERSECTION ");
#endif
                    break;
                case IntersectionState::NEARING_GENERIC:
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_ERROR("Cant switch from NEARING_FULL_STOP to NEARING_GENERIC ");
#endif
                    break;
            }
            break;
        case IntersectionState::STOP_NOW:
            switch (is) {
                case IntersectionState::ON_ROAD:
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_ERROR("Cant switch from STOP_NOW to ON_ROAD ");
#endif
                    break;
                case IntersectionState::NEARING_HAVE_WAY:
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_ERROR("Cant switch from STOP_NOW to NEARING_HAVE_WAY ");
#endif
                    break;
                case IntersectionState::NEARING_GIVE_WAY:
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_ERROR("Cant switch from STOP_NOW to NEARING_GIVE_WAY ");
#endif
                    break;
                case IntersectionState::NEARING_FULL_STOP:
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_ERROR("Cant switch from STOP_NOW to NEARING_FULL_STOP ");
#endif
                    break;
                case IntersectionState::STOP_NOW:
                    break;
                case IntersectionState::ON_INTERSECTION:
                    interState = is;
                    break;
                case IntersectionState::NEARING_GENERIC:
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_ERROR("Cant switch from STOP_NOW to NEARING_GENERIC ");
#endif
                    break;
            }
            break;
        case IntersectionState::ON_INTERSECTION:
            switch (is) {
                case IntersectionState::ON_ROAD:
                    maneuverEntry++;
                    updateTurnParkingLotAndManeuver();
                    interState = is;
                    break;
                case IntersectionState::NEARING_HAVE_WAY:
                    maneuverEntry++;
                    updateTurnParkingLotAndManeuver();
                    interState = is;
                    break;
                case IntersectionState::NEARING_GIVE_WAY:
                    maneuverEntry++;
                    updateTurnParkingLotAndManeuver();
                    interState = is;
                    break;
                case IntersectionState::NEARING_FULL_STOP:
                    maneuverEntry++;
                    updateTurnParkingLotAndManeuver();
                    interState = is;
                    break;
                case IntersectionState::STOP_NOW:
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_ERROR("Cant switch from ON_INTERSECTION to STOP_NOW ");
#endif
                    break;
                case IntersectionState::ON_INTERSECTION:
                    break;
                case IntersectionState::NEARING_GENERIC:
                    maneuverEntry++;
                    updateTurnParkingLotAndManeuver();
                    interState = is;
                    break;
            }
            break;
        case IntersectionState::NEARING_GENERIC:
            switch (is) {
                case IntersectionState::ON_ROAD:
#ifdef DEBUG_WORLD_FILTER_IN_LOG
                    LOG_ERROR("Cant switch from NEARING_GENERIC to ON_ROAD ");
#endif
                    break;
                case IntersectionState::NEARING_HAVE_WAY:
                    interState = is;
                    break;
                case IntersectionState::NEARING_GIVE_WAY:
                    interState = is;
                    break;
                case IntersectionState::NEARING_FULL_STOP:
                    interState = is;
                    break;
                case IntersectionState::STOP_NOW:
                    interState = is;
                    break;
                case IntersectionState::ON_INTERSECTION:
                    interState = is;
                    break;
                case IntersectionState::NEARING_GENERIC:
                    interState = is;
                    break;
            }
            break;
    }
}

bool WorldFilterIn::nextCheckpoint(const HTWKPoint &position, const TilePoint *pPoint, const tFloat32 &heading) {
    if (pPoint != nullptr) {
        switch (pPoint->edge) {
            case Orientation::Global::SOUTH:
                return position.y() >= pPoint->y();
            case Orientation::Global::WEST:
                return position.x() >= pPoint->x();
            case Orientation::Global::EAST:
                return position.x() < pPoint->x();
            case Orientation::Global::NORTH:
                return position.y() < pPoint->y();
        }
    }

    return false;
}

int WorldFilterIn::getTilesTillIntersection(TilePoint *pPoint) {
    TilePoint *nextPoint = pPoint;
    MapElement nextTile = map.getTileByTilePoint(nextPoint);

    int counterTillIntersection = 0;

    while (nextTile.getTileTypeId() != MapTileType::INTERSECTION_T &&
           nextTile.getTileTypeId() != MapTileType::INTERSECTION_PLUS) {
        if (nextPoint == nullptr ||
            nextPoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)] == nullptr) {
            counterTillIntersection = INT_MAX;
            break;
        }

        nextPoint = nextPoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)];
        nextTile = map.getTileByTilePoint(nextPoint);
        counterTillIntersection++;
    }

    return counterTillIntersection;
}

tResult WorldFilterIn::getParkinglotsFromFile() {
    cFilename fileParkinglots = GetPropertyStr(ROADSIGN_AND_PARKINGLOT_PROPERTY);
    ADTF_GET_CONFIG_FILENAME(fileParkinglots);
    fileParkinglots = fileParkinglots.CreateAbsolutePath(".");

    if (fileParkinglots.IsEmpty()) {
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    if (cFileSystem::Exists(fileParkinglots)) {
        tinyxml2::XMLDocument xmlDocument;

        tinyxml2::XMLError eResult = xmlDocument.LoadFile(fileParkinglots);

        if (!(eResult == tinyxml2::XMLError::XML_SUCCESS)) {
            LOG_ERROR("Can't open File");
            RETURN_ERROR(ERR_INVALID_FILE);
        }

        tinyxml2::XMLNode *pRoot = xmlDocument.FirstChild();

        //check root node
        if (pRoot == nullptr) {
            LOG_ERROR("No Root Element found");
            RETURN_ERROR(ERR_INVALID_FILE);
        }

        for (tinyxml2::XMLElement *xmlParkingSpaceElement = pRoot->NextSibling()->FirstChildElement();
             xmlParkingSpaceElement != nullptr; xmlParkingSpaceElement = xmlParkingSpaceElement->NextSiblingElement()) {
            tinyxml2::XMLElement *tmpXmlParkingSpaceElement = xmlParkingSpaceElement;

            if (std::strcmp(tmpXmlParkingSpaceElement->Value(), "parkingSpace") == 0) {
                int id;
                double x = 0;
                double y = 0;
                int status;
                int direction;

                std::setlocale(LC_ALL, "en_US.UTF-8"); //floatingpoint hotfix
                tmpXmlParkingSpaceElement->QueryIntAttribute("id", &id);
                tmpXmlParkingSpaceElement->QueryDoubleAttribute("x", &x);
                tmpXmlParkingSpaceElement->QueryDoubleAttribute("y", &y);
                tmpXmlParkingSpaceElement->QueryIntAttribute("status", &status);
                tmpXmlParkingSpaceElement->QueryIntAttribute("direction", &direction);
                std::setlocale(LC_ALL, "de_DE.UTF-8");

                parkingLots.emplace_back(
                        ParkingLot{id, HTWKPoint(x, y), status, static_cast<Orientation::MapTile>(direction)});
            }
        }

    }

    RETURN_NOERROR;
}

bool WorldFilterIn::tilePointIsDeadEnd(TilePoint *pPoint) {
    if(nullptr != pPoint) {
        return (pPoint->next[static_cast<int>(Orientation::TurnDirection::LEFT)] == nullptr &&
                pPoint->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)] == nullptr &&
                pPoint->next[static_cast<int>(Orientation::TurnDirection::RIGHT)] == nullptr);
    } else {
        return true;
    }
}

tResult WorldFilterIn::sendParkingLotUpdate(const ParkingLot& lot) {
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescParking->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {
        __adtf_sample_write_lock_mediadescription(m_pDescParking, pMediaSample, pCoder);
        if (!parkingIDS.IDsSet) {
            pCoder->GetID("i16Identifier", parkingIDS.m_szIDi16Identifier);
            pCoder->GetID("f32x", parkingIDS.m_szIDF32X);
            pCoder->GetID("f32y", parkingIDS.m_szIDF32Y);
            pCoder->GetID("ui16Status", parkingIDS.m_szIDUi16Status);
            parkingIDS.IDsSet = tTrue;
        }
        const tInt16 id = lot.id;
        const auto f32x = static_cast<tFloat32>(lot.position.x());
        const auto f32y = static_cast<tFloat32>(lot.position.y());
        const auto status = static_cast<tUInt16>(lot.status);
        pCoder->Set(parkingIDS.m_szIDi16Identifier, (tVoid *) &id);
        pCoder->Set(parkingIDS.m_szIDF32X, (tVoid *) &f32x);
        pCoder->Set(parkingIDS.m_szIDF32Y, (tVoid *) &f32y);
        pCoder->Set(parkingIDS.m_szIDUi16Status, (tVoid *) &status);
        pMediaSample->SetTime(_clock->GetTime());
    }
    RETURN_IF_FAILED(parkingUpdateOutputPin.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult WorldFilterIn::Start(__exception) {
    checkForParkingLotsIfDistanceMoved = GetPropertyFloat(DISTANCE_TO_BE_MOVED_CHECK_LOT);
    return cFilter::Start(__exception_ptr);
}
