//
// Created by lina on 02.10.17.
//

#include "GridMaps.h"

ADTF_FILTER_PLUGIN(OID_ADTF_FILTER_DEF, OID_ADTF_FILTER_DEF, GridMaps)

GridMaps::GridMaps(const tChar *__info) : cFilter(__info) {

    SetPropertyFloat(USONIC_MAX_DISTANCE_PROPERTY, USONIC_MAX_DISTANCE_DEFAULT);
    SetPropertyStr(USONIC_MAX_DISTANCE_PROPERTY NSSUBPROP_DESCRIPTION, USONIC_MAX_DISTANCE_DESCRIPTION);
    SetPropertyFloat(USONIC_MAX_DISTANCE_PROPERTY  NSSUBPROP_MIN, USONIC_MAX_DISTANCE_MIN);
    SetPropertyFloat(USONIC_MAX_DISTANCE_PROPERTY  NSSUBPROP_MAX, USONIC_MAX_DISTANCE_MAX);

    SetPropertyFloat(USONIC_MIN_DISTANCE_PROPERTY, USONIC_MIN_DISTANCE_DEFAULT);
    SetPropertyStr(USONIC_MIN_DISTANCE_PROPERTY NSSUBPROP_DESCRIPTION, USONIC_MIN_DISTANCE_DESCRIPTION);
    SetPropertyFloat(USONIC_MIN_DISTANCE_PROPERTY  NSSUBPROP_MIN, USONIC_MIN_DISTANCE_MIN);
    SetPropertyFloat(USONIC_MIN_DISTANCE_PROPERTY  NSSUBPROP_MAX, USONIC_MIN_DISTANCE_MAX);

    SetPropertyFloat(L_FREE_PROPERTY, L_FREE_DEFAULT);
    SetPropertyStr(L_FREE_PROPERTY NSSUBPROP_DESCRIPTION, L_FREE_DESCRIPTION);
    SetPropertyFloat(L_FREE_PROPERTY  NSSUBPROP_MIN, L_FREE_MIN);
    SetPropertyFloat(L_FREE_PROPERTY  NSSUBPROP_MAX, L_FREE_MAX);

    SetPropertyFloat(L_OCC_PROPERTY, L_OCC_DEFAULT);
    SetPropertyStr(L_OCC_PROPERTY NSSUBPROP_DESCRIPTION, L_OCC_DESCRIPTION);
    SetPropertyFloat(L_OCC_PROPERTY  NSSUBPROP_MIN, L_OCC_MIN);
    SetPropertyFloat(L_OCC_PROPERTY  NSSUBPROP_MAX, L_OCC_MAX);

    SetPropertyFloat(ALPHA_PROPERTY, ALPHA_DEFAULT);
    SetPropertyStr(ALPHA_PROPERTY NSSUBPROP_DESCRIPTION, ALPHA_DESCRIPTION);
    SetPropertyFloat(ALPHA_PROPERTY  NSSUBPROP_MIN, ALPHA_MIN);
    SetPropertyFloat(ALPHA_PROPERTY  NSSUBPROP_MAX, ALPHA_MAX);

    SetPropertyFloat(BETA_PROPERTY, BETA_DEFAULT);
    SetPropertyStr(BETA_PROPERTY NSSUBPROP_DESCRIPTION, BETA_DESCRIPTION);
    SetPropertyFloat(BETA_PROPERTY  NSSUBPROP_MIN, BETA_MIN);
    SetPropertyFloat(BETA_PROPERTY  NSSUBPROP_MAX, BETA_MAX);

    SetPropertyBool(LOG_GRID_PROPERTY, tFalse);

    gridMap = cv::Mat(GRID_MAP_HEIGHT, GRID_MAP_WIDTH, CV_8UC1, cv::Scalar::all(128));

    for (int i = 0; i < sizeof(occMap) / sizeof(*occMap); i++) {
        for (int j = 0; j < sizeof(occMap[i]) / sizeof(*occMap[i]); j++) {
            occMap[i][j] = L_O;
        }
    }

}

GridMaps::~GridMaps() = default;

tResult GridMaps::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {

        RETURN_IF_FAILED(CreateDescriptions(__exception_ptr));

        // create the input pin
        RETURN_IF_FAILED(uSonicStructInputPin.Create("uSonicStruct", ultrasonicStructMediaType,
                                                     static_cast<IPinEventSink *> (this)));
        RETURN_IF_FAILED(RegisterPin(&uSonicStructInputPin));

        RETURN_IF_FAILED(positionInputPin.Create("position", positionMediaType, static_cast<IPinEventSink *> (this)));
        RETURN_IF_FAILED(RegisterPin(&positionInputPin));

        RETURN_IF_FAILED(
                obstacleInput.Create("obstacles",
                                     new adtf::cMediaType(MEDIA_TYPE_TRACKING_DATA, MEDIA_SUBTYPE_TRACKING_DATA),
                                     static_cast<IPinEventSink *> (this)));
        RETURN_IF_FAILED(RegisterPin(&obstacleInput));

        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
    } else if (eStage == StageGraphReady) {
        heading = 0;
        pos_x = 0;
        pos_y = 0;
    } else if (eStage == StageNormal) {
        uSonicMaxDistance = (tFloat32) GetPropertyFloat(USONIC_MAX_DISTANCE_PROPERTY, USONIC_MAX_DISTANCE_DEFAULT);
        uSonicMinDistance = (tFloat32) GetPropertyFloat(USONIC_MIN_DISTANCE_PROPERTY, USONIC_MIN_DISTANCE_DEFAULT);
        lfree = (tFloat32) GetPropertyFloat(L_FREE_PROPERTY, L_FREE_DEFAULT);
        locc = (tFloat32) GetPropertyFloat(L_OCC_PROPERTY, L_OCC_DEFAULT);
        alphaValue = (tFloat32) GetPropertyFloat(ALPHA_PROPERTY, ALPHA_DEFAULT);
        betaValue = (int) GetPropertyInt(BETA_PROPERTY, BETA_DEFAULT);
        logToCSV = (tBool) GetPropertyBool(LOG_GRID_PROPERTY);
    }

    RETURN_NOERROR;
}

tResult GridMaps::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) {

    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

        // by comparing it to our member pin variable we can find out which pin received
        // the sample
        if (pSource == &uSonicStructInputPin) {
            //LOG_INFO(adtf_util::cString::Format("uSonicStruct Input received"));
            {
                // this will aquire the read lock on the sample and declare and initialize a pointer to the data
                __sample_read_lock(pMediaSample, tUltrasonicStruct, pData);
                // now we can access the sample data through the pointer
                receivedUSonicStruct = *pData;
            }

            if (positionInit) {
                sensorData = ultrasonicSensorPoseCalculation(heading, pos_x, pos_y, receivedUSonicStruct);
                updateGridMap(sensorData, pos_x, pos_y);

                cv::Mat flippedMat;
                cv::flip(gridMap, flippedMat, 0);
                gridMap = flippedMat;

                processVideo(flippedMat);
            }

        } else if (pSource == &positionInputPin) {
            //LOG_INFO(adtf_util::cString::Format("position Input Pin received"));
            {
                __adtf_sample_read_lock_mediadescription(positionDescription, pMediaSample, inputCoder);
                inputCoder->Get("f32x", (tVoid *) &pos_x);
                inputCoder->Get("f32y", (tVoid *) &pos_y);
                inputCoder->Get("f32heading", (tVoid *) &heading);
            }

            positionInit = true;

            //cv::putText(gridMap, "Hallo i bims 1 GridMap", cv::Point(50,50), FONT_HERSHEY_PLAIN, 1, cv::Scalar(0));
        } else if (pSource == &obstacleInput) {
            {
                __adtf_sample_read_lock(pMediaSample, tTrackingData, pData);
                htwk::recieveVector(pMediaSample, obstacleDataList);
            }
        }
    }

    RETURN_NOERROR;
}

tResult GridMaps::CreateDescriptions(IException **__exception_ptr) {
    cObjectPtr<IMediaDescriptionManager> descManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                         (tVoid **) &descManager, __exception_ptr));

    // get ultrasonicStructMediaType
    tChar const *ultrasonicStructDesc = descManager->GetMediaDescription("tUltrasonicStruct");
    RETURN_IF_POINTER_NULL(ultrasonicStructDesc);
    ultrasonicStructMediaType = new cMediaType(0, 0, 0, "tUltrasonicStruct", ultrasonicStructDesc,
                                               IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(
            ultrasonicStructMediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION,
                                                    (tVoid **) &ultrasonicStructDescription));

    //get tPosition
    tChar const *posDescription = descManager->GetMediaDescription("tPosition");
    RETURN_IF_POINTER_NULL(posDescription);
    positionMediaType = new cMediaType(0, 0, 0, "tPosition", posDescription,
                                       IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(positionMediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &positionDescription));

    RETURN_NOERROR;
}

tResult GridMaps::CreateOutputPins(IException **__exception_ptr) {
    //create Pin for Map Visualization Video
    RETURN_IF_FAILED(
            videoPin.Create("map_video", IPin::PD_Output, static_cast<IPinEventSink *>(this)));
    RETURN_IF_FAILED(RegisterPin(&videoPin));

    RETURN_NOERROR;
}

std::vector<ultrasonicSensorPose>
GridMaps::ultrasonicSensorPoseCalculation(tFloat32 heading, tFloat32 posX, tFloat32 posY,
                                          tUltrasonicStruct sensorValues) {
    //LOG_INFO(adtf_util::cString::Format("current IMU position: %f %f %f", posX, posY, (heading * 180/PI)));
    posX += GRID_MAP_WIDTH*0.1/2.0;
    posY += GRID_MAP_HEIGHT*0.1/2.0;
    vector<ultrasonicSensorPose> sensorVector;
    for (int i = 0; i < USONICSENSORCOUNT; ++i) {
        ultrasonicSensorPose pose{};
        auto sensor = static_cast<ultrasonicSensors>(i);
        switch (sensor) {
            case USONICMIDREAR:
                pose.pos.setXval(USONIC_MIDREAR_X + posX);
                pose.pos.setYval(USONIC_MIDREAR_Y + posY);
                pose.orientation = static_cast<tFloat32>(USONIC_MIDREAR_HEADING * PI / 180.0 + heading);
                pose.sensorValue = sensorValues.tRearCenter.f32Value;
                rotateSensorPoint(pose.pos, heading, HTWKPoint(posX, posY));
                //LOG_INFO(adtf_util::cString::Format("Sensorpos Mid Rear Test: %f %f %f",pose.pos.x(), pose.pos.y(),(pose.orientation * 180/PI)));
                break;
            case USONICRIGHTREAR:
                pose.pos.setXval(USONIC_RIGHTREAR_X + posX);
                pose.pos.setYval(USONIC_RIGHTREAR_Y + posY);
                pose.orientation = static_cast<tFloat32>(USONIC_RIGHTREAR_HEADING * PI / 180.0 + heading);
                pose.sensorValue = sensorValues.tRearRight.f32Value;
                rotateSensorPoint(pose.pos, heading, HTWKPoint(posX, posY));
                //LOG_INFO(adtf_util::cString::Format("Sensorpos Right Rear: %f %f %f",pose.pos.x(), pose.pos.y(),(pose.orientation * 180/PI)));
                break;
            case USONICLEFTREAR:
                pose.pos.setXval(USONIC_LEFTREAR_X + posX);
                pose.pos.setYval(USONIC_LEFTREAR_Y + posY);
                pose.orientation = static_cast<tFloat32>(USONIC_LEFTREAR_HEADING * PI / 180.0 + heading);
                pose.sensorValue = sensorValues.tRearLeft.f32Value;
                rotateSensorPoint(pose.pos, heading, HTWKPoint(posX, posY));
                //LOG_INFO(adtf_util::cString::Format("Sensorpos Left Rear: %f %f %f",pose.pos.x(), pose.pos.y(),(pose.orientation * 180/PI)));
                break;
            case USONICRIGHTMID:
                pose.pos.setXval(USONIC_RIGHTMID_X + posX);
                pose.pos.setYval(USONIC_RIGHTMID_Y + posY);
                pose.orientation = static_cast<tFloat32>(USONIC_RIGHTMID_HEADING * PI / 180.0 + heading);
                pose.sensorValue = sensorValues.tSideRight.f32Value;
                rotateSensorPoint(pose.pos, heading, HTWKPoint(posX, posY));
                //LOG_INFO(adtf_util::cString::Format("Sensorpos Right Mid: %f %f %f",pose.pos.x(), pose.pos.y(),(pose.orientation * 180/PI)));
                break;
            case USONICLEFTMID:
                pose.pos.setXval(USONIC_LEFTMID_X + posX);
                pose.pos.setYval(USONIC_LEFTMID_Y + posY);
                pose.orientation = static_cast<tFloat32>(USONIC_LEFTMID_HEADING * PI / 180.0 + heading);
                pose.sensorValue = sensorValues.tSideLeft.f32Value;
                rotateSensorPoint(pose.pos, heading, HTWKPoint(posX, posY));
                //LOG_INFO(adtf_util::cString::Format("Sensorpos Left Mid: %f %f %f",pose.pos.x(), pose.pos.y(),(pose.orientation * 180/PI)));
                break;
            case USONICMIDFRONT:
                pose.pos.setXval(USONIC_MIDFRONT_X + posX);
                pose.pos.setYval(USONIC_MIDFRONT_Y + posY);
                pose.orientation = static_cast<tFloat32>(USONIC_MIDFRONT_HEADING * PI / 180.0 + heading);
                pose.sensorValue = sensorValues.tFrontCenter.f32Value;
                rotateSensorPoint(pose.pos, heading, HTWKPoint(posX, posY));
                //LOG_INFO(adtf_util::cString::Format("Sensorpos Mid Front: %f %f %f",pose.pos.x(), pose.pos.y(),(pose.orientation * 180/PI)));
                break;
            case USONICMIDRIGHTFRONT:
                pose.pos.setXval(USONIC_MIDRIGHTFRONT_X + posX);
                pose.pos.setYval(USONIC_MIDRIGHTFRONT_Y + posY);
                pose.orientation = static_cast<tFloat32>(USONIC_MIDRIGHTFRONT_HEADING * PI / 180.0 + heading);
                pose.sensorValue = sensorValues.tFrontCenterRight.f32Value;
                rotateSensorPoint(pose.pos, heading, HTWKPoint(posX, posY));
                //LOG_INFO(adtf_util::cString::Format("Sensorpos Mid Right Front: %f %f %f",pose.pos.x(), pose.pos.y(),(pose.orientation * 180/PI)));
                break;
            case USONICMIDLEFTFRONT:
                pose.pos.setXval(USONIC_MIDLEFTFRONT_X + posX);
                pose.pos.setYval(USONIC_MIDLEFTFRONT_Y + posY);
                pose.orientation = static_cast<tFloat32>(USONIC_MIDLEFTFRONT_HEADING * PI / 180.0 + heading);
                pose.sensorValue = sensorValues.tFrontCenterLeft.f32Value;
                rotateSensorPoint(pose.pos, heading, HTWKPoint(posX, posY));
                //LOG_INFO(adtf_util::cString::Format("Sensorpos Mid Left Front: %f %f %f",pose.pos.x(), pose.pos.y(),(pose.orientation * 180/PI)));
                break;
            case USONICRIGHTFRONT:
                pose.pos.setXval(USONIC_RIGHTFRONT_X + posX);
                pose.pos.setYval(USONIC_RIGHTFRONT_Y + posY);
                pose.orientation = static_cast<tFloat32>(USONIC_RIGHTFRONT_HEADING * PI / 180.0 + heading);
                pose.sensorValue = sensorValues.tFrontRight.f32Value;
                rotateSensorPoint(pose.pos, heading, HTWKPoint(posX, posY));
                //LOG_INFO(adtf_util::cString::Format("Sensorpos Right Front: %f %f %f",pose.pos.x(), pose.pos.y(),(pose.orientation * 180/PI)));
                break;
            case USONICLEFTFRONT:
                pose.pos.setXval(USONIC_LEFTFRONT_X + posX);
                pose.pos.setYval(USONIC_LEFTFRONT_Y + posY);
                pose.orientation = static_cast<tFloat32>(USONIC_LEFTFRONT_HEADING * PI / 180.0 + heading);
                pose.sensorValue = sensorValues.tFrontLeft.f32Value;
                rotateSensorPoint(pose.pos, heading, HTWKPoint(posX, posY));
                //LOG_INFO(adtf_util::cString::Format("Sensorpos Left Front: %f %f %f",pose.pos.x(), pose.pos.y(),(pose.orientation * 180/PI)));
                break;
            default:
                break;
        }


        sensorVector.emplace_back(pose);
        /*for(const auto &element:sensorVector){
            LOG_INFO(adtf_util::cString::Format("hallo i bims 1 sensorposition: %f %f %f",element.pos.x(), element.pos.y(),element.orientation));
        }*/
    }

    return sensorVector;
}

void GridMaps::rotateSensorPoint(HTWKPoint &sensorPoint, const float &orientation, const HTWKPoint &IMUPosition) {
    float s = sin(orientation);
    float c = cos(orientation);

    // translate point back to origin:
    sensorPoint = sensorPoint.sub(IMUPosition);

    // rotate point
    HTWKPoint newPoint(sensorPoint.x() * c - sensorPoint.y() * s, sensorPoint.x() * s + sensorPoint.y() * c);

    // translate point back:
    sensorPoint = newPoint.add(IMUPosition);
}

void GridMaps::updateGridMap(const std::vector<ultrasonicSensorPose> &sensorPos, tFloat32 carX, tFloat32 carY) {
    carX += GRID_MAP_WIDTH*0.1/2.0;
    carY += GRID_MAP_HEIGHT*0.1/2.0;
    for (int x = 0; x < GRID_MAP_WIDTH; x++) {
        for (int y = 0; y < GRID_MAP_HEIGHT; y++) {
            HTWKPoint gridCell;

            //LOG_INFO(adtf_util::cString::Format("inspected grid cell x: %d y: %d", x, y));
            gridCell.setXval((x + 0.5) * 0.1);
            gridCell.setYval((y + 0.5) * 0.1);
            //LOG_INFO(adtf_util::cString::Format("mapped grid cell x: %f y: %f", gridCell.x(), gridCell.y()));
            if (sqrt(pow(gridCell.x() - carX, 2) + pow(gridCell.y() - carY, 2)) < uSonicMaxDistance) {
                float calcOcc = static_cast<float>(occMap[y][x] + inverseSensorModel(gridCell, sensorPos) - L_O);

                if (debugThisCell) {
                    //LOG_INFO(adtf_util::cString::Format("calcOcc: %f", calcOcc));
                    debugThisCell = false;
                }

                occMap[y][x] = calcOcc;
            }
            //LOG_INFO(adtf_util::cString::Format("new Occupancy: %d of grid x: %d y: %d", gridMap.at<std::uint8_t>(x,y), x, y));
        }
    }

    // write trackedObjects to Grid
    for(const auto &trackedObject: obstacleDataList) {

        //LOG_INFO(adtf_util::cString::Format("Tracked Object X: %f, Y: %f", trackedObject.position.x, trackedObject.position.y));

        HTWKPoint objectPositionInGrid (trackedObject.position.x * 10 + GRID_MAP_WIDTH / 2.0,
                                        trackedObject.position.y * 10 + GRID_MAP_HEIGHT / 2.0);

        auto trackedObjectStartX = static_cast<int>(objectPositionInGrid.x() - (trackedObject.radius * 10));
        auto trackedObjectStartY = static_cast<int>(objectPositionInGrid.y() - (trackedObject.radius * 10));

        auto trackedObjectEndX = static_cast<int>(objectPositionInGrid.x() + (trackedObject.radius * 10));
        auto trackedObjectEndY = static_cast<int>(objectPositionInGrid.y() + (trackedObject.radius * 10));

        //LOG_INFO(adtf_util::cString::Format("TrackedObject Start X: %d", trackedObjectStartX));
        //LOG_INFO(adtf_util::cString::Format("TrackedObject End X: %d", trackedObjectEndX));

        for(int x = trackedObjectStartX; x < trackedObjectEndX; x++) {
            for(int y = trackedObjectStartY; y < trackedObjectEndY; y++) {
                occMap[y][x] = L_TRACKED_OBJECTS;
            }
        }
    }

    std::string filename = "testGrid" + std::to_string(csvCounter) + ".csv";

    updateGrayValues();

    if(logToCSV && (csvCounter % 30 == 0)) {
        writeCSV(filename, gridMap);
    }
    csvCounter++;
}

double GridMaps::inverseSensorModel(HTWKPoint gridCellCenter, std::vector<ultrasonicSensorPose> sensorPos) {
    double allSensorPhis[USONICSENSORCOUNT];
    double allSensorRanges[USONICSENSORCOUNT];
    double r = 0;
    double phi = 0;
    double beta = betaValue * PI / 180;
    int sensorNumber = 0;

    double zk = 0;
    double zmin = uSonicMinDistance;
    double zmax = uSonicMaxDistance;

    double minDelta = -1;
    double thetaK = 0;

    for (int i = 0; i < USONICSENSORCOUNT; ++i) {
        allSensorRanges[i] = sqrt(pow(gridCellCenter.x() - sensorPos.at(i).pos.x(), 2) +
                                  pow(gridCellCenter.y() - sensorPos.at(i).pos.y(), 2));
        allSensorPhis[i] = atan2(gridCellCenter.y() - sensorPos.at(i).pos.y(),
                                 gridCellCenter.x() - sensorPos.at(i).pos.x());

        if (debugThisCell) {
//            LOG_INFO(adtf_util::cString::Format(
//                    "allSensorRanges i: %f, allSensorPhis i: %f sensorPos.at(i).orientation: %f", allSensorRanges[i],
//                    allSensorPhis[i], sensorPos.at(i).orientation));
        }

        if (fabs(allSensorPhis[i] - sensorPos.at(i).orientation) < minDelta || minDelta == -1) {
            r = allSensorRanges[i];
            phi = allSensorPhis[i];
            zk = sensorPos.at(i).sensorValue * 0.01;
            thetaK = sensorPos.at(i).orientation;
            sensorNumber = i;
            minDelta = fabs(allSensorPhis[i] - thetaK);
            if (debugThisCell) {
//                LOG_INFO(adtf_util::cString::Format("minDelta: %f", minDelta));
            }
        }
    }

    if (debugThisCell) {
//        LOG_INFO(adtf_util::cString::Format("Sensor: %d zk: %f", sensorNumber, zk));
//        LOG_INFO(adtf_util::cString::Format("r: %f phi: %f thetaK: %f minDelta: %f", r, phi, thetaK, minDelta));
//        LOG_INFO(adtf_util::cString::Format("gridCellCenter x: %f y: %f", gridCellCenter.x(), gridCellCenter.y()));
//        LOG_INFO(adtf_util::cString::Format("sensorPos x: %f y: %f", sensorPos.at(sensorNumber).pos.x(),
//                                            sensorPos.at(sensorNumber).pos.y()));

        debugThisCell = false;
    }

    if (r > min(zmax, zk + alphaValue / 2) || fabs(phi - thetaK) > beta / 2 || zk < zmin) {
        //LOG_INFO(adtf_util::cString::Format("returns l0"));
        //LOG_INFO(adtf_util::cString::Format("Sensor: %d zk: %f", sensorNumber, zk));
        return L_O; //doesn't change
    } else if (zk < zmax && fabs(r - zk) < alphaValue / 2) {
        //LOG_INFO(adtf_util::cString::Format("returns locc"));
        return locc; // occupied
    } else if (r <= zk) {
        //LOG_INFO(adtf_util::cString::Format("returns lfree"));
        return lfree; // free
    }

    return L_O;
}

int GridMaps::grayGridMapValue(double occValue) {
    double p = 1 - 1 / (1 + exp(occValue));
    int grayValue = (int) ((1 - p) * 255);
    //LOG_INFO(adtf_util::cString::Format("p: %f occValue: %f", p, occValue));
    return grayValue;
}

void GridMaps::updateGrayValues() {
    for (int i = 0; i < sizeof(occMap) / sizeof(*occMap); i++) {
        for (int j = 0; j < sizeof(occMap[i]) / sizeof(*occMap[i]); j++) {
            gridMap.at<std::uint8_t>(j, i) = static_cast<uint8_t>(grayGridMapValue(occMap[j][i]));
        }
    }
}

tResult GridMaps::processVideo(cv::Mat &image) {
    updateOutputImageFormat(image, outputBitmapFormat, videoPin);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(outputBitmapFormat.nSize));
    RETURN_IF_FAILED(pMediaSample->Update(_clock->GetTime(), image.data, outputBitmapFormat.nSize, IMediaSample::MSF_None));
    RETURN_IF_FAILED(videoPin.Transmit(pMediaSample));
    RETURN_NOERROR;

}


void GridMaps::writeCSV(string filename, cv::Mat m) {
    std::fstream outputFile;
    outputFile.open(filename, std::ios::out ) ;

    for(int i=0; i<m.rows; i++) {
        for(int j=0; j<m.cols; j++)
        {
            outputFile << static_cast<int>(m.at<std::uint8_t>(j, i)) << ", ";
        }
        outputFile << endl;

    }
    outputFile.close( );
}