#include "MapView.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, MapView)

MapView::MapView(const tChar *__info) : Leaf(__info) {
#ifdef DEBUG_VIDEO
    SetPropertyStr(STRAIGHT_IMAGE_PROPERTY, "straight.tiff");
    SetPropertyBool(STRAIGHT_IMAGE_PROPERTY
                            NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr(STRAIGHT_IMAGE_PROPERTY
                           NSSUBPROP_FILENAME
                           NSSUBSUBPROP_EXTENSIONFILTER, "TIFF Files (*.tiff)");
    SetPropertyStr(STRAIGHT_IMAGE_PROPERTY
                           NSSUBPROP_DESCRIPTION, "Picture to visulaize a straight tile");

    SetPropertyStr(S_TURN_PROPERTY_LEFT, "s_turn_left.tiff");
    SetPropertyBool(S_TURN_PROPERTY_LEFT
                            NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr(S_TURN_PROPERTY_LEFT
                           NSSUBPROP_FILENAME
                           NSSUBSUBPROP_EXTENSIONFILTER, "TIFF Files (*.tiff)");
    SetPropertyStr(S_TURN_PROPERTY_LEFT
                           NSSUBPROP_DESCRIPTION, "Picture to visulaize a s-turn left");

    SetPropertyStr(S_TURN_PROPERTY_RIGHT, "s_turn_right.tiff");
    SetPropertyBool(S_TURN_PROPERTY_RIGHT
                            NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr(S_TURN_PROPERTY_RIGHT
                           NSSUBPROP_FILENAME
                           NSSUBSUBPROP_EXTENSIONFILTER, "TIFF Files (*.tiff)");
    SetPropertyStr(S_TURN_PROPERTY_RIGHT
                           NSSUBPROP_DESCRIPTION, "Picture to visulaize a s-turn right");

    SetPropertyStr(TURN_SMALL_IMAGE_PROPERTY, "turn_small.tiff");
    SetPropertyBool(TURN_SMALL_IMAGE_PROPERTY
                            NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr(TURN_SMALL_IMAGE_PROPERTY
                           NSSUBPROP_FILENAME
                           NSSUBSUBPROP_EXTENSIONFILTER, "TIFF Files (*.tiff)");
    SetPropertyStr(TURN_SMALL_IMAGE_PROPERTY
                           NSSUBPROP_DESCRIPTION, "Picture to visulaize a small turn tile");

    SetPropertyStr(TURN_LARGE_IMAGE_PROPERTY, "turn_large.tiff");
    SetPropertyBool(TURN_LARGE_IMAGE_PROPERTY
                            NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr(TURN_LARGE_IMAGE_PROPERTY
                           NSSUBPROP_FILENAME
                           NSSUBSUBPROP_EXTENSIONFILTER, "TIFF Files (*.tiff)");
    SetPropertyStr(TURN_LARGE_IMAGE_PROPERTY
                           NSSUBPROP_DESCRIPTION, "Picture to visulaize a large turn tile");

    SetPropertyStr(CROSSING_T_IMAGE_PROPERTY, "crossing_t.tiff");
    SetPropertyBool(CROSSING_T_IMAGE_PROPERTY
                            NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr(CROSSING_T_IMAGE_PROPERTY
                           NSSUBPROP_FILENAME
                           NSSUBSUBPROP_EXTENSIONFILTER, "TIFF Files (*.tiff)");
    SetPropertyStr(CROSSING_T_IMAGE_PROPERTY
                           NSSUBPROP_DESCRIPTION, "Picture to visulaize a T-crossing tile");

    SetPropertyStr(CROSSING_PLUS_IMAGE_PROPERTY, "crossing_plus.tiff");
    SetPropertyBool(CROSSING_PLUS_IMAGE_PROPERTY
                            NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr(CROSSING_PLUS_IMAGE_PROPERTY
                           NSSUBPROP_FILENAME
                           NSSUBSUBPROP_EXTENSIONFILTER, "TIFF Files (*.tiff)");
    SetPropertyStr(CROSSING_PLUS_IMAGE_PROPERTY
                           NSSUBPROP_DESCRIPTION, "Picture to visulaize a Plus-crossing tile");

    SetPropertyStr(LOT_IMAGE_PROPERTY, "lot.tiff");
    SetPropertyBool(LOT_IMAGE_PROPERTY
                            NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr(LOT_IMAGE_PROPERTY
                           NSSUBPROP_FILENAME
                           NSSUBSUBPROP_EXTENSIONFILTER, "TIFF Files (*.tiff)");
    SetPropertyStr(LOT_IMAGE_PROPERTY
                           NSSUBPROP_DESCRIPTION, "Picture to visulaize a parking lot tile");
#endif

    SetPropertyBool(BOOL_PROPERTY, tFalse);
    SetPropertyBool(BOOL_PROPERTY NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(BOOL_PROPERTY NSSUBPROP_DESCRIPTION, "Change lane yes or no");


    SetPropertyFloat(MAX_RADIUS_PROPERTY, MAXIMUM_RADIUS_FOR_TRACK_DRIVING);
    SetPropertyStr(MAX_RADIUS_PROPERTY NSSUBPROP_DESCRIPTION, "Max radius until failure");

    mapInit = false;
    laneChangeTimer = 0;
}

MapView::~MapView() {
}

tResult MapView::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(Leaf::Init(eStage, __exception_ptr));

    if (StageFirst == eStage) {
        //load tile images for visualization

#ifdef DEBUG_VIDEO
        if (!tileImagesLoaded) {
            cFilename path;
            path = GetPropertyStr(STRAIGHT_IMAGE_PROPERTY);
            loadTileImage(straightTile, path);
            path = GetPropertyStr(TURN_SMALL_IMAGE_PROPERTY);
            loadTileImage(turnSmallTile, path);
            path = GetPropertyStr(TURN_LARGE_IMAGE_PROPERTY);
            loadTileImage(turnLargeTile, path);
            path = GetPropertyStr(CROSSING_PLUS_IMAGE_PROPERTY);
            loadTileImage(plusCrossingTile, path);
            path = GetPropertyStr(CROSSING_T_IMAGE_PROPERTY);
            loadTileImage(tCrossingTile, path);
            path = GetPropertyStr(LOT_IMAGE_PROPERTY);
            loadTileImage(parkingLotTile, path);
            path = GetPropertyStr(S_TURN_PROPERTY_LEFT);
            loadTileImage(s_turn_left, path);
            path = GetPropertyStr(S_TURN_PROPERTY_RIGHT);
            loadTileImage(s_turn_right, path);
            tileImagesLoaded = tTrue;
        }

        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
#endif

    } else if (StageNormal == eStage) {

    } else if (StageGraphReady == eStage) {


        maxRadius = static_cast<tFloat32>(GetPropertyFloat(MAX_RADIUS_PROPERTY));
    }

    RETURN_NOERROR;
}

tResult MapView::Shutdown(tInitStage eStage, __exception) {
    return Leaf::Shutdown(eStage, __exception_ptr);
}

tResult MapView::calcTrackTrajectory() {

    if (currentTilePoint != nullptr && nextTilePoint != nullptr) {
        map.getTrajectoryBetweenPoints(currentTilePoint, nextTilePoint, 0.15, trajectoryPoints);
    }

    if (nextTilePoint != nullptr && nextNextTilePoint != nullptr) {
        map.getTrajectoryBetweenPoints(nextTilePoint, nextNextTilePoint, 0.15, trajectoryPoints);
    }

    if (nextNextTilePoint != nullptr && nextNextNextTilePoint != nullptr) {
        map.getTrajectoryBetweenPoints(nextNextTilePoint, nextNextNextTilePoint, 0.15, trajectoryPoints);
    }

    RETURN_NOERROR;

}

tResult MapView::calcTrackTrajectoryOppositeLane() {
    if (currentTilePoint != nullptr && nextTilePoint != nullptr && nextNextTilePoint != nullptr &&
        nextNextNextTilePoint != nullptr) {
        TilePoint *currentTilePointOpposite;
        TilePoint *nextTilePointOpposite;
        TilePoint *nextNextTilePointOpposite;

        currentTilePointOpposite = map.getTilePointOppositeLane(currentTilePoint);
        nextTilePointOpposite = map.getTilePointOppositeLane(nextTilePoint);
        nextNextTilePointOpposite = map.getTilePointOppositeLane(nextNextTilePoint);

        map.getTrajectoryBetweenPoints(nextTilePointOpposite, currentTilePointOpposite, 0.15, trajectoryPoints);
        map.getTrajectoryBetweenPoints(nextNextTilePointOpposite, nextTilePointOpposite, 0.15, trajectoryPoints);
    }

    RETURN_NOERROR;
}

tResult MapView::findSteering(cv::Mat &mat, const HTWKPoint &position, const tFloat32 &orientation) {
    CarTrajectory trajectoryCalculator;
    std::vector<HTWKPoint> carTrajecotryPoints;

    float steering = -30;
    std::pair<float, double> bestTrajectory(-30, DBL_MAX);

    while (steering < 30) {
        carTrajecotryPoints.clear();
        trajectoryCalculator.getTrajectoryPoints(steering, position, orientation, 0.15, carTrajecotryPoints);
        double score = trajectoryCalculator.getTrajectoryScore(carTrajecotryPoints, trajectoryPoints);

        if (score < bestTrajectory.second) {
            bestTrajectory.first = steering;
            bestTrajectory.second = score;
        }

        steering += 1;
    }

    SetSteeringOutput(bestTrajectory.first);

#ifdef DEBUG_VIDEO
    drawCarTrajectory(mat, position, orientation, trajectoryCalculator, carTrajecotryPoints, bestTrajectory);
#endif

    RETURN_NOERROR;
}

tResult MapView::OnTrigger() {
    trajectoryPoints.clear();

    std::vector<MapElement> mapElementsVector;
    if (!IS_OK(worldService->Pull(WORLD_MAP, mapElementsVector))) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_ERROR("Map could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    map = TrackMap(mapElementsVector);

    if (!mapInit) {
        map.setTileLinks();
        mapInit = true;
    }

    if (!IS_OK(worldService->Pull(WORLD_INTERSECTION_STATE, interState))) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_ERROR("Inter state could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

//    if (!IS_OK(worldService->PullGridMap(gridMap))) {
//        LOG_ERROR("Gridmap could not be pulled from WorldService");
//    }

/*    if(isOffRoad) {
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }*/

    if (!IS_OK(worldService->Pull(WORLD_NEXT_TURN, nextTurn))) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_ERROR("Next turn could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(WORLD_CURRENT_POSITION, currentPosition))) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_ERROR("Current position could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

//    if (!IS_OK(worldService->Pull(WORLD_CURRENT_SPEED, currentSpeed))) {
//#ifdef DEBUG_MAP_VIEW_LOG
//        LOG_ERROR("Current speed could not be pulled from WorldService");
//#endif
//        TransmitStatus(BT::FAIL);
//        RETURN_ERROR(ERR_FAILED);
//    }

    if (!IS_OK(worldService->Pull(WORLD_CURRENT_HEADING, currentOrientation))) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_ERROR("Current Orientation could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(WORLD_CURRENT_TILE_POINT, currentTilePoint))) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_ERROR("Current tile point could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(WORLD_NEXT_TILE_POINT, nextTilePoint))) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_ERROR("Next tilepoint could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(WORLD_NEXT_NEXT_TILE_POINT, nextNextTilePoint))) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_ERROR("Next next tilepoint could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(WORLD_NEXT_NEXT_NEXT_TILE_POINT, nextNextNextTilePoint))) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_ERROR("Next next next tilepoint could not be pulled from WorldService");
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

    if (!IS_OK(worldService->Pull(WORLD_OBSTACLES, obstacleDataList))) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_ERROR("Obstacles could not be pulled from WorldService");
#endif
    }

    if (!IS_OK(worldService->Pull(THIS_IS_THE_POLICE, policeData))) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_ERROR("PoliceData could not be pulled");
#endif
    }

    if (!pullOut) {
        if (!IS_OK(worldService->Pull(CAR_AVOID_MOVE, pullOut))) {
#ifdef DEBUG_MAP_VIEW_LOG
            LOG_ERROR("Pull out could not be pulled from WorldService");
#endif
        }
    }


    if (_clock->GetStreamTime() > laneChangeTimer && laneChangeTimer != 0) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_INFO("set pull out false");
#endif
        pullOut = tFalse;
        laneChangeTimer = 0;
        worldService->Push(CAR_AVOID_MOVE, pullOut);
    }

    // TODO in ChangeSpeed triggern, aber Logik hier
    if (!pullOut || interState != IntersectionState::ON_ROAD) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_INFO("dont pull Out");
#endif
        calcTrackTrajectory();
        laneChangeTimer = 0;
    } else {
        if (laneChangeTimer == 0) {
            laneChangeTimer = _clock->GetStreamTime() + DRIVE_ON_OTHER_LANE;
        }
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_INFO("pull out");
#endif
        calcTrackTrajectoryOppositeLane();
    }

    tFloat32 radius;
    if (!IS_OK(worldService->Pull(WORLD_CURRENT_RADIUS, radius))) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_ERROR("Radius could not be pulled from WorldService");
#endif
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    //if postion is uncertain on road dont use track driving
    if (!pullOut && radius > maxRadius && interState == IntersectionState::ON_ROAD) {
        TransmitStatus(BT::FAIL);
        RETURN_NOERROR;
    }

#ifdef DEBUG_MAP_VIEW_LOG
    LOG_INFO("MapDriving active!");
#endif

#ifdef DEBUG_VIDEO
    buildMapImage();
    markCurrentTilePosition(currentPosition, currentOrientation);

    drawTrackGraph(mapImage);
    drawTrackGraphFromPosition(mapImage, currentPosition, currentOrientation, 4);
#endif

#ifdef DEBUG_VIDEO
    drawTrackTrajectory(mapImage, trajectoryPoints);
#endif

    findSteering(mapImage, currentPosition, currentOrientation * 180 / PI);
    setTurnSignal(interState, nextTurn);

#ifdef DEBUG_VIDEO

    if (currentTilePoint != nullptr) {
        printTilePoint(mapImage, currentTilePoint);
    }

    if (nextTilePoint != nullptr) {
        printTilePoint(mapImage, nextTilePoint);
    }

    if (nextNextTilePoint != nullptr) {
        printTilePoint(mapImage, nextNextTilePoint);
    }

    if (nextNextNextTilePoint != nullptr) {
        printTilePoint(mapImage, nextNextNextTilePoint);
    }

    printObstacles();

    //flip image so the origin ist bottom left
    cv::Mat flippedMat;
    cv::flip(mapImage, flippedMat, 0);
    mapImage = flippedMat;

    processVideo(mapImage);
    //map.clear();

    //have to be resetted since the map can change
    originCorrectionX = 0;
    originCorrectionY = 0;
#endif

    TransmitStatus(BT::RUNNING);
    RETURN_NOERROR;
}



void MapView::setTurnSignal(IntersectionState state, Orientation::TurnDirection direction) {
    switch (interState) {
        case IntersectionState::ON_ROAD:
            SwitchLights(TURNSIGNAL_LEFT, tFalse);
            SwitchLights(TURNSIGNAL_RIGHT, tFalse);
            break;
        case IntersectionState::NEARING_HAVE_WAY:
            switch (nextTurn) {
                case Orientation::TurnDirection::STRAIGHT:
                    SwitchLights(TURNSIGNAL_LEFT, tFalse);
                    SwitchLights(TURNSIGNAL_RIGHT, tFalse);
                    break;
                case Orientation::TurnDirection::LEFT:
                    SwitchLights(TURNSIGNAL_RIGHT, tFalse);
                    SwitchLights(TURNSIGNAL_LEFT, tTrue);
                    break;
                case Orientation::TurnDirection::RIGHT:
                    SwitchLights(TURNSIGNAL_LEFT, tFalse);
                    SwitchLights(TURNSIGNAL_RIGHT, tTrue);
                    break;
            }
            break;
        case IntersectionState::NEARING_GIVE_WAY:
            switch (nextTurn) {
                case Orientation::TurnDirection::STRAIGHT:
                    SwitchLights(TURNSIGNAL_LEFT, tFalse);
                    SwitchLights(TURNSIGNAL_RIGHT, tFalse);
                    break;
                case Orientation::TurnDirection::LEFT:
                    SwitchLights(TURNSIGNAL_RIGHT, tFalse);
                    SwitchLights(TURNSIGNAL_LEFT, tTrue);
                    break;
                case Orientation::TurnDirection::RIGHT:
                    SwitchLights(TURNSIGNAL_LEFT, tFalse);
                    SwitchLights(TURNSIGNAL_RIGHT, tTrue);
                    break;
            }
            break;
        case IntersectionState::NEARING_FULL_STOP:
            switch (nextTurn) {
                case Orientation::TurnDirection::STRAIGHT:
                    SwitchLights(TURNSIGNAL_LEFT, tFalse);
                    SwitchLights(TURNSIGNAL_RIGHT, tFalse);
                    break;
                case Orientation::TurnDirection::LEFT:
                    SwitchLights(TURNSIGNAL_RIGHT, tFalse);
                    SwitchLights(TURNSIGNAL_LEFT, tTrue);
                    break;
                case Orientation::TurnDirection::RIGHT:
                    SwitchLights(TURNSIGNAL_LEFT, tFalse);
                    SwitchLights(TURNSIGNAL_RIGHT, tTrue);
                    break;
            }
            break;
        case IntersectionState::STOP_NOW:
            switch (nextTurn) {
                case Orientation::TurnDirection::STRAIGHT:
                    SwitchLights(TURNSIGNAL_LEFT, tFalse);
                    SwitchLights(TURNSIGNAL_RIGHT, tFalse);
                    break;
                case Orientation::TurnDirection::LEFT:
                    SwitchLights(TURNSIGNAL_RIGHT, tFalse);
                    SwitchLights(TURNSIGNAL_LEFT, tTrue);
                    break;
                case Orientation::TurnDirection::RIGHT:
                    SwitchLights(TURNSIGNAL_LEFT, tFalse);
                    SwitchLights(TURNSIGNAL_RIGHT, tTrue);
                    break;
            }
            break;
        case IntersectionState::ON_INTERSECTION:
            switch (nextTurn) {
                case Orientation::TurnDirection::STRAIGHT:
                    SwitchLights(TURNSIGNAL_LEFT, tFalse);
                    SwitchLights(TURNSIGNAL_RIGHT, tFalse);
                    break;
                case Orientation::TurnDirection::LEFT:
                    SwitchLights(TURNSIGNAL_RIGHT, tFalse);
                    SwitchLights(TURNSIGNAL_LEFT, tTrue);
                    break;
                case Orientation::TurnDirection::RIGHT:
                    SwitchLights(TURNSIGNAL_LEFT, tFalse);
                    SwitchLights(TURNSIGNAL_RIGHT, tTrue);
                    break;
            }
            break;
        case IntersectionState::NEARING_GENERIC:
            switch (nextTurn) {
                case Orientation::TurnDirection::STRAIGHT:
                    SwitchLights(TURNSIGNAL_LEFT, tFalse);
                    SwitchLights(TURNSIGNAL_RIGHT, tFalse);
                    break;
                case Orientation::TurnDirection::LEFT:
                    SwitchLights(TURNSIGNAL_RIGHT, tFalse);
                    SwitchLights(TURNSIGNAL_LEFT, tTrue);
                    break;
                case Orientation::TurnDirection::RIGHT:
                    SwitchLights(TURNSIGNAL_LEFT, tFalse);
                    SwitchLights(TURNSIGNAL_RIGHT, tTrue);
                    break;
            }
            break;
    }
}

#ifdef DEBUG_VIDEO

tResult MapView::CreateOutputPins(IException **__exception_ptr) {
    //create Pin for Map Visulaization Video
    RETURN_IF_FAILED(
            videoPin.Create("map_video", IPin::PD_Output, static_cast<IPinEventSink *>(this)));
    RETURN_IF_FAILED(RegisterPin(&videoPin));

    RETURN_NOERROR;
}

tResult MapView::buildMapImage() {
    //create canvas for map tiles
    cv::Mat tmpMat(map.getHeightInMeters() * METERS_TO_PIXELS, map.getWidthInMeters() * METERS_TO_PIXELS,
                   CV_8UC3,
                   cv::Scalar::all(255));
#ifdef DEBUG_MAP_VIEW_LOG
    LOG_INFO("TmpMat created");
#endif
    //calculate correction for the image origin
    {
        tInt32 minX = map.getMinX();
        tInt32 minY = map.getMinY();

        originCorrectionX = -minX;
        originCorrectionY = -minY;
    }

    //iterate through map and place tiles on canvas
    for (tUInt64 i = 0; i < map.asVector().size(); i++) {
        switch (map.asVector().at(i).getTileTypeId()) {
            case MapTileType::STRAIGHT:
                insertTile(straightTile, tmpMat, map.asVector().at(i));
                break;
            case MapTileType::INTERSECTION_T:
                insertTile(tCrossingTile, tmpMat, map.asVector().at(i));
                break;
            case MapTileType::INTERSECTION_PLUS:
                insertTile(plusCrossingTile, tmpMat, map.asVector().at(i));
                break;
            case MapTileType::TURN_LARGE:
                insertTile(turnLargeTile, tmpMat, map.asVector().at(i));
                break;
            case MapTileType::TURN_SMALL:
                insertTile(turnSmallTile, tmpMat, map.asVector().at(i));
                break;
            case MapTileType::LOT:
                insertTile(parkingLotTile, tmpMat, map.asVector().at(i));
                break;
            case MapTileType::TURN_S_LEFT:
                insertTile(s_turn_left, tmpMat, map.asVector().at(i));
                break;
            case MapTileType::TURN_S_RIGHT:
                insertTile(s_turn_right, tmpMat, map.asVector().at(i));
                break;
        }

    }

/*    //draw tile entry-points
    for (tUInt64 i = 0; i < map.size(); i++) {
        printTileEntryPoints(tmpMat, map[i]);
    }

    for (tUInt64 i = 0; i < map.size(); i++) {
        printTileExitPoints(tmpMat, map[i]);
    }
*/

    mapImage = tmpMat;

    RETURN_NOERROR;
}

tResult MapView::processVideo(cv::Mat &image) {
    UpdateOutputImageFormat(image);

    cImage newImage;
    newImage.Create(outputBitmapFormat.nWidth, outputBitmapFormat.nHeight, outputBitmapFormat.nBitsPerPixel,
                    outputBitmapFormat.nBytesPerLine, image.data);

    //create the new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));
    //updating media sample
    RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(),
                                          IMediaSample::MSF_None));
    //transmitting
    RETURN_IF_FAILED(videoPin.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult MapView::loadTileImage(cv::Mat &image, cFilename &path) {
    ADTF_GET_CONFIG_FILENAME(path);
    path = path.CreateAbsolutePath(".");

    // check if given property is not empty
    if (path.IsEmpty()) {
#ifdef DEBUG_MAP_VIEW_LOG
        LOG_ERROR("Can't find Image");
#endif
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    image = cv::imread(cv::String(path), CV_LOAD_IMAGE_COLOR);   // Read the file

    /*if (image.empty()) // Check for invalid input
    {
    #ifdef DEBUG_MAP_VIEW_LOG
        LOG_ERROR("Invalid file");
        #endif
        RETURN_ERROR(ERR_INVALID_FILE);
    }*/

    RETURN_NOERROR;
}

void MapView::UpdateOutputImageFormat(const cv::Mat &outputImage) {
    //check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != outputBitmapFormat.nSize) {
        Mat2BmpFormat(outputImage, outputBitmapFormat);
        //set output format for output pin
        videoPin.SetFormat(&outputBitmapFormat, nullptr);
    }
}

void MapView::insertTile(cv::Mat &tileMat, cv::Mat &imageMat, const MapElement &element) {
    cv::Mat tileToInsert = tileMat.clone();

    //rotate tile according to Orientation
    switch (element.getOrient()) {
        case Orientation::MapTile::NORTH:
            cv::rotate(tileToInsert, tileToInsert, cv::ROTATE_90_COUNTERCLOCKWISE);
            break;
        case Orientation::MapTile::WEST:
            cv::rotate(tileToInsert, tileToInsert, cv::ROTATE_180);
            break;
        case Orientation::MapTile::EAST:
            break;
        case Orientation::MapTile::SOUTH:
            cv::rotate(tileToInsert, tileToInsert, cv::ROTATE_90_CLOCKWISE);
            break;
    }

    //coordinate system of image is y-down, trackmap system is y-up
    //tiles have to be flipped
    cv::Mat flippedMat;
    cv::flip(tileToInsert, flippedMat, 0);
    tileToInsert = flippedMat;

    auto imageX = toCVPointInImage(element.getPositon()).x;
    auto imageY = toCVPointInImage(element.getPositon()).y;

    //insert tile
    //coordinates have to be adjusted from origin at bottom/left to top/left
    tileToInsert.copyTo(imageMat(cv::Rect(imageX, imageY, tileToInsert.cols, tileToInsert.rows)));
}

tResult MapView::markCurrentTilePosition(HTWKPoint position, tFloat32 orientation) {
    MapElement *me = map.getTileByPosition(position);

    if (me != nullptr) {
        cv::Point tileOriginOnMat(toCVPointInImage(me->getPositon()));

        cv::Point tileEndOnMat(
                static_cast<int>(tileOriginOnMat.x + me->getTileSize() * METERS_TO_PIXELS),
                static_cast<int>(tileOriginOnMat.y + me->getTileSize() * METERS_TO_PIXELS));

        cv::rectangle(mapImage, tileOriginOnMat, tileEndOnMat, cv::Scalar(0, 0, 255), 10);

        HTWKPoint positionWithCarsize = position.moveNewPointByDistanceAndAngleRad(WHEEL_BASE / 2.0f, orientation);
        cv::RotatedRect rRect = cv::RotatedRect(toCVPointInImage(positionWithCarsize),
                                                cv::Size2f(48, 30), orientation * 180 / PI);

        cv::Point2f vertices[4];
        rRect.points(vertices);
        cv::Point2i verticesInterger[4];
        verticesInterger[0] = vertices[0];
        verticesInterger[1] = vertices[1];
        verticesInterger[2] = vertices[2];
        verticesInterger[3] = vertices[3];

        cv::Scalar color = cv::Scalar(0,0,0);
        //Police seen, last seen in last 10 secs
        if(policeData.countSeen > 0 && (policeData.lastSeen - _clock->GetTime()) * 1e-6 < 10.0){
            color =  policeData.lastSeen % 2 == 0 ? cv::Scalar(255,0,0) : cv::Scalar(0,0,255);
        }
        cv::fillConvexPoly(mapImage, verticesInterger, 4, cv::Scalar(0, 0, 0));
    }

    RETURN_NOERROR;
}

cv::Point MapView::toCVPointInImage(const HTWKPoint &point) {
    return cv::Point(static_cast<int>((point.x() + originCorrectionX) * METERS_TO_PIXELS),
                     static_cast<int>((point.y() + originCorrectionY) * METERS_TO_PIXELS));
}

void MapView::printTileEntryPoints(cv::Mat &mat, MapElement &mapElement) {
    for (const auto &point: mapElement.getTileEntryPoints()) {
        if(point != nullptr) {
            cv::circle(mat, toCVPointInImage(*point), 4, cv::Scalar(255, 0, 0), -1);
        }
        //std::cout << "Draw Entrypoint " << toCVPointInImage(*point) << std::endl;
    }
}

void MapView::printTileExitPoints(cv::Mat &mat, MapElement &mapElement) {
    for (const auto &point: mapElement.getTileEntryPoints()) {
        if(point != nullptr) {
            cv::circle(mat, toCVPointInImage(*point), 4, cv::Scalar(255, 153, 0), -1);
        }
        //std::cout << "Draw Exitpoint " << toCVPointInImage(*point) << std::endl;

    }
}

tResult MapView::drawTrackGraph(cv::Mat &mat) {
    for (auto &tile: map.asVector()) {
        for (const auto entryPoint: tile.getTileEntryPoints()) {
            if (entryPoint != nullptr && entryPoint->next != nullptr) {
                for (const auto next : entryPoint->next) {
                    if (next != nullptr) {
                        cv::line(mat, toCVPointInImage(*entryPoint), toCVPointInImage(*next), cv::Scalar(0, 76, 153),
                                 2);
                        /*std::cout << "Draw from Point " << toCVPointInImage(*entryPoint) << " to Point "
                                  << toCVPointInImage(*next) << " on Edge " << static_cast<int>(next->edge) << std::endl;*/
                    }
                }
            }
        }
    }

    RETURN_NOERROR;
}

tResult MapView::drawTrackTrajectory(cv::Mat &image, const std::vector<HTWKPoint> &trajectoryPoints) {

    for (const auto &point: trajectoryPoints) {
        cv::circle(image, toCVPointInImage(point), 4, cv::Scalar(255, 0, 0), -1);
    }

    RETURN_NOERROR;
}

void MapView::drawCarTrajectory(const cv::Mat &mat, const HTWKPoint &position, const tFloat32 &orientation,
                                CarTrajectory &trajectoryCalculator, vector<HTWKPoint> &carTrajecotryPoints,
                                const pair<float, double> &bestTrajectory) {
    carTrajecotryPoints.clear();
    trajectoryCalculator.getTrajectoryPoints(bestTrajectory.first, position, orientation, 0.15, carTrajecotryPoints);
    for (auto const &point: carTrajecotryPoints) {
        circle(mat, toCVPointInImage(point), 3, cv::Scalar::all(0), -1);
    }
}

tResult
MapView::drawTrackGraphFromPosition(cv::Mat &image, const HTWKPoint &position, const float &Orientation,
                                    const int &lookAheadDistance) {

    cv::circle(image, toCVPointInImage(*currentTilePoint), 4, cv::Scalar(255, 0, 0), -1);

    std::vector<TilePoint *> tilePointPtrs;
    tilePointPtrs.push_back(currentTilePoint);

    for (int i = 0; i < lookAheadDistance; i++) {
        std::vector<TilePoint *> currentTilePointPtrsGeneration = tilePointPtrs;

        for (const auto &tilePointPtr: currentTilePointPtrsGeneration) {
            if (tilePointPtr->next[static_cast<int>(Orientation::TurnDirection::LEFT)] != nullptr) {
                cv::line(image, toCVPointInImage(*tilePointPtr),
                         toCVPointInImage(*tilePointPtr->next[static_cast<int>(Orientation::TurnDirection::LEFT)]),
                         cv::Scalar(20, 255, 57), 2);

                tilePointPtrs.insert(tilePointPtrs.begin(),
                                     tilePointPtr->next[static_cast<int>(Orientation::TurnDirection::LEFT)]);
            }

            if (tilePointPtr->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)] != nullptr) {
                cv::line(image, toCVPointInImage(*tilePointPtr),
                         toCVPointInImage(*tilePointPtr->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)]),
                         cv::Scalar(20, 255, 57), 2);

                tilePointPtrs.insert(tilePointPtrs.begin(),
                                     tilePointPtr->next[static_cast<int>(Orientation::TurnDirection::STRAIGHT)]);
            }

            if (tilePointPtr->next[static_cast<int>(Orientation::TurnDirection::RIGHT)] != nullptr) {
                cv::line(image, toCVPointInImage(*tilePointPtr),
                         toCVPointInImage(*tilePointPtr->next[static_cast<int>(Orientation::TurnDirection::RIGHT)]),
                         cv::Scalar(20, 255, 57), 2);

                tilePointPtrs.insert(tilePointPtrs.begin(),
                                     tilePointPtr->next[static_cast<int>(Orientation::TurnDirection::RIGHT)]);
            }
        }

        if(!tilePointPtrs.empty()) {
            tilePointPtrs.pop_back();
        }
    }

    RETURN_NOERROR;
}

void MapView::printTilePoint(cv::Mat &mat, TilePoint *pPoint) {

    if (pPoint != nullptr) {
        switch (pPoint->edge) {
            case Orientation::Global::NORTH:
                cv::circle(mapImage, toCVPointInImage(*pPoint), 8, cv::Scalar(255, 0, 0), -1);
                break;
            case Orientation::Global::WEST:
                cv::circle(mapImage, toCVPointInImage(*pPoint), 8, cv::Scalar(0, 255, 0), -1);
                break;
            case Orientation::Global::EAST:
                cv::circle(mapImage, toCVPointInImage(*pPoint), 8, cv::Scalar(0, 0, 255), -1);
                break;
            case Orientation::Global::SOUTH:
                cv::circle(mapImage, toCVPointInImage(*pPoint), 8, cv::Scalar(0, 255, 255), -1);
                break;
        }
    }
}

void MapView::printObstacles() {
    for (const auto &obstacleData: obstacleDataList) {
        const auto pos = toCVPointInImage(HTWKPoint(obstacleData.position.x, obstacleData.position.y));
        cv::Scalar color;
        auto radius = static_cast<int>(obstacleData.radius * 100);
        switch (obstacleData.type) {
            case ObstacleType::CAR:
                color = cv::Scalar(255, 0, 0);
                break;
            case ObstacleType::PYLON:
                color = cv::Scalar(0, 0, 255);
                break;
            case ObstacleType::SIGN:
                color = cv::Scalar(125, 125, 125);
                break;
            case ObstacleType::CHILD:
                color = cv::Scalar(0, 255, 0);
                radius = static_cast<int>(obstacleData.radius * 140);
                break;
            case ObstacleType::ADULT:
                color = cv::Scalar(0, 150, 0);
                radius = static_cast<int>(obstacleData.radius * 120);
                break;
            default:
                color = cv::Scalar(0, 0, 0);
                break;
        }
        cv::circle(mapImage, pos, radius, color, -1);
    }
}

#endif