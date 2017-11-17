#include "MarkerPositioningFilter.h"

ADTF_FILTER_PLUGIN(OID_ADTF_FILTER_DEF, OID_ADTF_FILTER_DEF, MarkerPositioningFilter)

MarkerPositioningFilter::MarkerPositioningFilter(const tChar *__info) : cFilter(__info) {
    SetPropertyStr("Configuration", "roadSign.xml");
    SetPropertyBool("Configuration" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configuration" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("Configuration" NSSUBPROP_DESCRIPTION, "Configuration file for the roadsign coordinates");

    SetPropertyFloat(MP_PROP_CAMERA_OFFSET_LAT, 0.0f);
    SetPropertyBool(MP_PROP_CAMERA_OFFSET_LAT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MP_PROP_CAMERA_OFFSET_LAT NSSUBPROP_DESCRIPTION, "Camera offset in lateral direction");

    SetPropertyFloat(MP_PROP_CAMERA_OFFSET_LON, 0.3f);
    SetPropertyBool(MP_PROP_CAMERA_OFFSET_LON NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MP_PROP_CAMERA_OFFSET_LON NSSUBPROP_DESCRIPTION, "Camera offset in longitudinal direction");

    SetPropertyFloat(MP_PROP_SPEED_SCALE, 1.0f);
    SetPropertyBool(MP_PROP_SPEED_SCALE NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MP_PROP_SPEED_SCALE NSSUBPROP_DESCRIPTION, "Initial scale value for the speed measurement");

    SetPropertyFloat(PROPERTY_MAX_ANGLE, PROPERTY_DEFAULT_ANGLE);
    SetPropertyBool(PROPERTY_MAX_ANGLE NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PROPERTY_MAX_ANGLE NSSUBPROP_DESCRIPTION, "Maximum Angle for recognition in degree");
    SetPropertyFloat(PROPERTY_MAX_ANGLE NSSUBPROP_MIN, 30.0);
    SetPropertyFloat(PROPERTY_MAX_ANGLE NSSUBPROP_MAX, 65.0);

    SetPropertyFloat(PROPERTY_MIN_MOV_DIST, PROPERTY_DEFAULT_MIN_MOV_DIST);
    SetPropertyBool(PROPERTY_MIN_MOV_DIST NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PROPERTY_MIN_MOV_DIST NSSUBPROP_DESCRIPTION,
                   "Minimum moved Distance in Metern for new Missign Sign Check");
    SetPropertyFloat(PROPERTY_MIN_MOV_DIST NSSUBPROP_MIN, 0.0);
    SetPropertyFloat(PROPERTY_MIN_MOV_DIST NSSUBPROP_MAX, 2.0);

    SetPropertyFloat(PROPERTY_CHECK_OFFSET_X, PROPERTY_DEFAULT_CHECK_OFFSET_X);
    SetPropertyBool(PROPERTY_CHECK_OFFSET_X NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PROPERTY_CHECK_OFFSET_X NSSUBPROP_DESCRIPTION, "Offset from Car X in Metern");
    SetPropertyFloat(PROPERTY_CHECK_OFFSET_X NSSUBPROP_MIN, 0.0);
    SetPropertyFloat(PROPERTY_CHECK_OFFSET_X NSSUBPROP_MAX, 2.0);

    SetPropertyFloat(PROPERTY_CHECK_OFFSET_Y, PROPERTY_DEFAULT_CHECK_OFFSET_Y);
    SetPropertyBool(PROPERTY_CHECK_OFFSET_Y NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PROPERTY_CHECK_OFFSET_Y NSSUBPROP_DESCRIPTION, "Offset from Car Y in Metern");
    SetPropertyFloat(PROPERTY_CHECK_OFFSET_Y NSSUBPROP_MIN, 0.0);
    SetPropertyFloat(PROPERTY_CHECK_OFFSET_Y NSSUBPROP_MAX, 2.0);

    SetPropertyFloat(PROPERTY_RADIUS_MISSING_SIGN, PROPERTY_DEFAULT_RADIUS_MISSING_SIGN);
    SetPropertyBool(PROPERTY_RADIUS_MISSING_SIGN NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PROPERTY_RADIUS_MISSING_SIGN NSSUBPROP_DESCRIPTION, "Radius for Missing Sign Check");
    SetPropertyFloat(PROPERTY_RADIUS_MISSING_SIGN NSSUBPROP_MIN, 0.1);
    SetPropertyFloat(PROPERTY_RADIUS_MISSING_SIGN NSSUBPROP_MAX, 2.0);

    SetPropertyInt(PROPERTY_NOISE_COUNT, PROPERTY_DEFAULT_NOISE_COUNT);
    SetPropertyBool(PROPERTY_NOISE_COUNT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PROPERTY_NOISE_COUNT NSSUBPROP_DESCRIPTION,
                   "Inital noisy markers will be dropped until value is reached");
    SetPropertyInt(PROPERTY_NOISE_COUNT NSSUBPROP_MIN, 0);
    SetPropertyInt(PROPERTY_NOISE_COUNT NSSUBPROP_MAX, 100);

    SetPropertyInt(PROPERTY_MARKER_COUNT, PROPERTY_DEFAULT_MARKER_COUNT);
    SetPropertyBool(PROPERTY_MARKER_COUNT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PROPERTY_MARKER_COUNT NSSUBPROP_DESCRIPTION, "Repositioning of marker is found more then count");
    SetPropertyInt(PROPERTY_MARKER_COUNT NSSUBPROP_MIN, 0);
    SetPropertyInt(PROPERTY_MARKER_COUNT NSSUBPROP_MAX, 20);
}

tResult MarkerPositioningFilter::CreateInputPins(__exception) {
    cObjectPtr<IMediaType> pInputType;
    RETURN_IF_FAILED(
            AllocMediaType(&pInputType, MEDIA_TYPE_ROAD_SIGN_DATA, MEDIA_SUBTYPE_ROAD_SIGN_DATA, __exception_ptr));
    RETURN_IF_FAILED(inputRoadSignsPin.Create("RoadSigns", pInputType, this));
    RETURN_IF_FAILED(RegisterPin(&inputRoadSignsPin));

    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                         (tVoid **) &pDescManager, __exception_ptr));

    tChar const *strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,
                                                             IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &descMeasSpeed));
    RETURN_IF_FAILED(inputSpeedPin.Create("Speed", pTypeSignalValue, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&inputSpeedPin));

    tChar const *strDescInerMeasUnit = pDescManager->GetMediaDescription("tInerMeasUnitData");
    RETURN_IF_POINTER_NULL(strDescInerMeasUnit);
    cObjectPtr<IMediaType> pTypeInerMeasUnit = new cMediaType(0, 0, 0, "tInerMeasUnitData", strDescInerMeasUnit,
                                                              IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeInerMeasUnit->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION,
                                                     (tVoid **) &m_pDescriptionInerMeasUnitData));
    RETURN_IF_FAILED(
            inputIMUPin.Create("IMU", pTypeInerMeasUnit, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&inputIMUPin));

    RETURN_NOERROR;
}

tResult MarkerPositioningFilter::CreateOutputPins(__exception) {
    cObjectPtr<IMediaType> pOutputType;
    RETURN_IF_FAILED(
            AllocMediaType(&pOutputType, MEDIA_TYPE_INITAL_ROAD_SIGN_DATA, MEDIA_SUBTYPE_INITAL_ROAD_SIGN_DATA,
                           __exception_ptr));
    RETURN_IF_FAILED(outputInitalRoadSignsPin.Create("initalRoadSigns", pOutputType, this));
    RETURN_IF_FAILED(RegisterPin(&outputInitalRoadSignsPin));

    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                         (tVoid **) &pDescManager, __exception_ptr));

    tChar const *strDescPosition = pDescManager->GetMediaDescription("tPosition");
    RETURN_IF_POINTER_NULL(strDescPosition);
    cObjectPtr<IMediaType> pTypePosition = new cMediaType(0, 0, 0, "tPosition", strDescPosition,
                                                          IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    // create the position OutputPin
    RETURN_IF_FAILED(outputPositionPin.Create("Position", pTypePosition, this));
    RETURN_IF_FAILED(RegisterPin(&outputPositionPin));
    // set the description for the extended marker pin
    RETURN_IF_FAILED(pTypePosition->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &m_pDescPosition));

    tChar const *strDescTrafficSignUpdate = pDescManager->GetMediaDescription("tTrafficSign");
    RETURN_IF_POINTER_NULL(strDescTrafficSignUpdate);
    cObjectPtr<IMediaType> pTypeTrafficSignUpdate = new cMediaType(0, 0, 0, "tTrafficSign", strDescTrafficSignUpdate,
                                                                   IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    // create the position OutputPin
    RETURN_IF_FAILED(outputChangedRoadSignPin.Create("TrafficSignUpdate", pTypeTrafficSignUpdate, this));
    RETURN_IF_FAILED(RegisterPin(&outputChangedRoadSignPin));
    // set the description for the extended marker pin
    RETURN_IF_FAILED(pTypeTrafficSignUpdate->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION,
                                                          (tVoid **) &m_pDescTrafficSignUpdate));

    RETURN_NOERROR;
}

tResult MarkerPositioningFilter::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst) {
        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
    } else if (eStage == StageNormal) {
    } else if (eStage == StageGraphReady) {
        RETURN_IF_FAILED(thread.Create(NULL, 0, cThread::TS_Suspended, static_cast<IThreadFunc *>(this)));
    }
    RETURN_NOERROR;
}

tResult MarkerPositioningFilter::Start(__exception) {
    ReadProperties(NULL);
    if (loadConfiguration() == ERR_INVALID_FILE) {
        THROW_ERROR_DESC(ERR_INVALID_FILE, "RoadSigns couldn't be loaded");
    }
    initalRoadSignsSend = false;

    // initialize EKF variables
    m_state = cv::Mat(6, 1, CV_64F, cv::Scalar::all(0));
    m_errorCov = cv::Mat(6, 6, CV_64F, cv::Scalar::all(0));
    double T = 0.1;
    m_errorCov[0][0] = MP_PROCESS_INIT_X;
    m_errorCov[1][1] = MP_PROCESS_INIT_Y;
    m_errorCov[2][2] = MP_PROCESS_INIT_HEADING;
    m_errorCov[2][3] = MP_PROCESS_INIT_HEADING / T;
    m_errorCov[3][3] = MP_PROCESS_INIT_HEADING_DRIFT;
    m_errorCov[4][4] = MP_PROCESS_INIT_SPEED;
    m_errorCov[4][5] = MP_PROCESS_INIT_SPEED / T;
    m_errorCov[5][5] = MP_PROCESS_INIT_SPEED_SCALE;

    m_transitionMatrix = cv::Mat(6, 6, CV_64F, cv::Scalar::all(0));
    setIdentity(m_transitionMatrix);

    m_processCov = cv::Mat(6, 6, CV_64F, cv::Scalar::all(0));
    m_processCov[0][0] = MP_PROCESS_X;
    m_processCov[1][1] = MP_PROCESS_Y;
    m_processCov[2][2] = MP_PROCESS_HEADING;
    m_processCov[3][3] = MP_PROCESS_HEADING_DRIFT;
    m_processCov[4][4] = MP_PROCESS_SPEED;
    m_processCov[5][5] = MP_PROCESS_SPEED_SCALE;

    m_isInitialized = tFalse;
    timeStamp = 0;
    speed = 0;
    noiseCount = 0;
    wheelSpeedIDsSet = false;
    imuIDs.m_bIDsInerMeasUnitSet = false;
    positionIDS.positionIDsSet = false;
    trafficSignUpdateIDs.traffigSignUpdateIDsSet = false;
    return cFilter::Start(__exception_ptr);
}

tResult MarkerPositioningFilter::Stop(__exception) {
    if (thread.GetState() == cThread::TS_Running) {
        thread.Suspend(tTrue);
    }
    return cFilter::Stop(__exception_ptr);
}

tResult MarkerPositioningFilter::Shutdown(tInitStage eStage, __exception) {
    if (eStage == StageNormal) {
        thread.Terminate(tTrue);
        thread.Release();
    }
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult MarkerPositioningFilter::ThreadFunc(cThread *Thread, tVoid *data, tSize size) {
    const float correctedHeading = currentPos.heading - M_PI_2;
    //Wir schauen x-Meter vorne rechts vor uns
    cv::Point2f center = htwk::translateAndRotate2DPoint(checkMissignSignOffset, correctedHeading, cv::Point2f(0, 0),
                                                         currentPos.pt);
    //Vergleichen mit einer Distanz-Funktionen ob Schild im Umkreis liegt
    for (auto &sign : initalRoadSigns) {
        if (sign.sign.id != ROAD_SIGN::MISSING) {
            const double xd = center.x - sign.sign.pos.x;
            const double yd = center.y - sign.sign.pos.y;
            const auto dist = static_cast<float>(sqrt(xd * xd + yd * yd));
            //Check Schild im Umkreis mit korrektem Heading
            if (dist < checkRadius && abs(angleDiff(sign.sign.heading, currentPos.heading)) < maxAngle) {
                if (sign.count == MARKER_NOT_SEEN) {
#ifdef DEBUG_MARKER_POS_LOG
                    LOG_INFO(adtf_util::cString::Format("MISSING SIGN %s (%f , %f) Dir: %f Head: %f",
                                                        mapTypeToString(sign.sign.id).c_str(), sign.sign.pos.x,
                                                        sign.sign.pos.y, sign.sign.heading * RAD2DEG,
                                                        currentPos.heading * RAD2DEG));
#endif
                    sign.sign.id = ROAD_SIGN::MISSING;
                    transmitUpdateTrafficSign(sign);
                }
                break;
            }
        }
    }
    oldPos = currentPos;
    RETURN_IF_FAILED(thread.Suspend());
    RETURN_NOERROR;
}

tResult MarkerPositioningFilter::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
                                            IMediaSample *pMediaSample) {
    __synchronized_obj(m_critSecOnPinEvent);
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        if (!initalRoadSignsSend) {
            transmitInitalRoadSigns();
            initalRoadSignsSend = true;
        }
        // process RoadSignExt sample
        if (pSource == &inputRoadSignsPin && speedInit && imuInit) {
            tRoadSignDetectionResult roadSign;
            {//------------------------------------------
                __adtf_sample_read_lock(pMediaSample, tRoadSignDetectionResult, pData);
                memcpy(&roadSign, pData, pMediaSample->GetSize());
            }//------------------
            RETURN_IF_FAILED(ProcessRoadSigns(roadSign));

        } else if (pSource == &inputSpeedPin) {
            {
                __adtf_sample_read_lock_mediadescription(descMeasSpeed, pMediaSample, pCoderInput);
                if (!wheelSpeedIDsSet) {
                    pCoderInput->GetID("f32Value", speedID);
                    pCoderInput->GetID("ui32ArduinoTimestamp", speedArduinoTimestamp);
                    wheelSpeedIDsSet = tTrue;
                }
                pCoderInput->Get(speedID, (tVoid *) &speed);
                //tUInt32 time = 0;
                //pCoderInput->Get(speedArduinoTimestamp, (tVoid *) &time);
            }

            speedInit = true;
        } else if (pSource == &inputIMUPin) {
            RETURN_IF_FAILED(ProcessIMU(pMediaSample));
            imuInit = true;
        }
    }
    RETURN_NOERROR;
}

/*! Calculates orientation, distance and pose of the given road sign,
 * and updates the positioning filter accordingly */
tResult MarkerPositioningFilter::ProcessRoadSigns(const tRoadSignDetectionResult &sign) {
    if (noiseCount < noiseCountMax) {
        noiseCount++;
        RETURN_NOERROR;
    }
    const tFloat32 lateral = sign.tVec.x + cameraOffsetLat;
    const tFloat32 longitudinal = sign.tVec.z + cameraOffsetLon;
    const tFloat32 d0 = sqrt(lateral * lateral + longitudinal * longitudinal);
    tFloat32 a0 = atan2(lateral, longitudinal);
    a0 = -static_cast<tFloat32 >(normalizeAngle(a0, 0.0f) * RAD2DEG);
    // normalize angle -pi:pi and change direction
    // check angle and distance limit
    if (fabs(a0) > MP_LIMIT_ALPHA || d0 > MP_LIMIT_DISTANCE) {
        RETURN_NOERROR;
    }
    if (!m_isInitialized) {
        initViaStartMarker(sign, d0, a0);
        RETURN_NOERROR;
    }
    tFloat64 dt = 0;
    const int index = findMatchingRoadSign(sign, d0, a0, dt);
    checkChangedOrNewSign(sign, index);

    // #1 no matching marker found
    // #2 too long time from previous marker input
    // #3 dropping samples when a new marker is found
    if (index < 0 || dt > 0.3 || initalRoadSigns[index].count < markerCountRepositioning) {
        RETURN_NOERROR;
    }
    updateEKF(d0, a0, index);
    RETURN_NOERROR;
}

void MarkerPositioningFilter::checkChangedOrNewSign(const tRoadSignDetectionResult &sign, const int &index) {
#ifdef DEBUG_MARKER_POS_LOG
    const tFloat32 lateral = sign.tVec.x + cameraOffsetLat;
    const tFloat32 longitudinal = sign.tVec.z + cameraOffsetLon;
    const tFloat32 d0 = sqrt(lateral * lateral + longitudinal * longitudinal);
    tFloat32 a0 = atan2(lateral, longitudinal);
    a0 = -static_cast<tFloat32 >(normalizeAngle(a0, 0.0f) * RAD2DEG);
    auto heading = static_cast<tFloat32>(m_state[2][0] + a0 * DEG2RAD);
    heading = normalizeAngle(heading, 0.0f);
#endif
    // Sign not found
    if (index == -1) {
#ifdef DEBUG_MARKER_POS_LOG
        const auto x0 = static_cast<tFloat32>(m_state[0][0] + cos(heading) * d0);
        const auto y0 = static_cast<tFloat32>(m_state[1][0] + sin(heading) * d0);
        LOG_INFO(adtf_util::cString::Format("NOT FOUND %s (%f , %f) Head: %f", mapTypeToString(sign.id).c_str(), x0, y0,
                                            heading * RAD2DEG));
#endif
    } else if (initalRoadSigns[index].sign.id == sign.id && initalRoadSigns[index].count <= MARKER_SEEN) { //First seen
        initalRoadSigns[index].count++;
        //transmitUpdateTrafficSign(initalRoadSigns[index]);
#ifdef DEBUG_MARKER_POS_LOG
        LOG_INFO(cString::Format("FOUND %s (%f , %f) Dir: %f Head: %f", mapTypeToString(sign.id).c_str(),
                                 initalRoadSigns[index].sign.pos.x, initalRoadSigns[index].sign.pos.y,
                                 initalRoadSigns[index].sign.heading * RAD2DEG, heading * RAD2DEG));
#endif
    } else if (initalRoadSigns[index].sign.id != sign.id) { //changed
#ifdef DEBUG_MARKER_POS_LOG
        LOG_INFO(cString::Format("CHANGED %s to %s (%f , %f) Dir: %f Head: %f",
                                 mapTypeToString(initalRoadSigns[index].sign.id).c_str(),
                                 mapTypeToString(sign.id).c_str(), initalRoadSigns[index].sign.pos.x,
                                 initalRoadSigns[index].sign.pos.y,
                                 initalRoadSigns[index].sign.heading * RAD2DEG, heading * RAD2DEG));
#endif
        initalRoadSigns[index].sign.id = sign.id;
        transmitUpdateTrafficSign(initalRoadSigns[index]);
        if (initalRoadSigns[index].count <= MARKER_SEEN) {
            initalRoadSigns[index].count = 1;
        }
    }
}

int MarkerPositioningFilter::findMatchingRoadSign(const tRoadSignDetectionResult &sign, const tFloat32 &d0,
                                                  const tFloat32 &a0, tFloat64 &dt) {
    int index = -1;
    // calculate heading wrt marker
    auto heading = static_cast<tFloat32>(m_state[2][0] + a0 * DEG2RAD);
    heading = normalizeAngle(heading, 0.0f);
    // estimate marker location based on current vehicle location
    // and marker measurement
    const auto x0 = static_cast<tFloat32>(m_state[0][0] + cos(heading) * d0);
    const auto y0 = static_cast<tFloat32>(m_state[1][0] + sin(heading) * d0);

    for (size_t i = 0; i < initalRoadSigns.size(); i++) {
        // calculate error distance
        const tFloat32 dx = x0 - initalRoadSigns[i].sign.pos.x;
        const tFloat32 dy = y0 - initalRoadSigns[i].sign.pos.y;
        const tFloat32 distance = sqrt(dx * dx + dy * dy);

        int found = false;
        //re-Init on init-sign
        if (sign.id == initalRoadSigns[i].sign.id && initalRoadSigns[i].sign.init) {
            // estimate location
            tFloat32 x = initalRoadSigns[i].sign.pos.x + cos(heading) * -d0;
            tFloat32 y = initalRoadSigns[i].sign.pos.y + sin(heading) * -d0;

            // initialize filter but keep heading states
            m_state.at<double>(0) = x;
            m_state.at<double>(1) = y;

            m_state.at<double>(4) = 0;
            m_state.at<double>(5) = 0;

            //re-Init all Markers
            for (auto &marker : initalRoadSigns) {
                marker.count = MARKER_NOT_SEEN;
                marker.ticks = 0;
            }
            found = true;
            // a marker found within the radius with correct direction
        } else if (distance < initalRoadSigns[i].sign.radius &&
                   abs(angleDiff(initalRoadSigns[i].sign.heading, heading)) < maxAngle) {
            found = true;
        }
        if (found) {
            if (initalRoadSigns[i].ticks != 0) {
                // calculate time from previous marker measurement
                dt = static_cast<tFloat32>((GetTime() - initalRoadSigns[i].ticks) * 1e-6);
                // reset sample counter when marker reappears
                if (dt > 1.0f) {
                    initalRoadSigns[i].count = MARKER_SEEN;
                }
            }
            initalRoadSigns[i].ticks = GetTime();
            index = i;
            initalRoadSigns[index].count++;
            break;
        }
    }
    return index;
}

void MarkerPositioningFilter::updateEKF(const tFloat32 &d0, const tFloat32 &a0, const int &ind) {
    // calculate heading update
    // create pseudo measurement for heading update
    const auto headingUpdate = static_cast<tFloat32>(m_state[2][0]);
    const tFloat32 shift = -d0; // reversed translation direction

    // calculate translation direction
    tFloat32 correction = headingUpdate + a0 * DEG2RAD;
    correction = normalizeAngle(correction, 0.0f);

    // update location estimate
    const tFloat32 x = initalRoadSigns[ind].sign.pos.x + cos(correction) * shift;
    const tFloat32 y = initalRoadSigns[ind].sign.pos.y + sin(correction) * shift;

    cv::Mat1d measCov = cv::Mat(3, 3, CV_64F, cv::Scalar::all(0));
    measCov[0][0] = MP_MEASUREMENT_X;
    measCov[1][1] = MP_MEASUREMENT_Y;
    measCov[2][2] = MP_MEASUREMENT_HEADING;

    cv::Mat1d measurementMatrix = cv::Mat(3, 6, CV_64F, cv::Scalar::all(0));
    measurementMatrix[0][0] = 1.0f;
    measurementMatrix[1][1] = 1.0f;
    measurementMatrix[2][2] = 1.0f;

    cv::Mat1d identity = cv::Mat(6, 6, CV_64F, cv::Scalar::all(0));
    setIdentity(identity);

    cv::Mat1d measurement = cv::Mat(3, 1, CV_64F, cv::Scalar::all(0));
    measurement[0][0] = x;
    measurement[1][0] = y;
    measurement[2][0] = headingUpdate;

    // propagate covariance
    m_errorCov = m_transitionMatrix * m_errorCov * m_transitionMatrix.t() + m_processCov;

    // calculate innovation
    cv::Mat1d innovation = measurement - measurementMatrix * m_state;
    // modulo of the heading measurement
    innovation[2][0] = static_cast<double>(angleDiff(
            mod(static_cast<tFloat32>(m_state[2][0]), 2.0f * static_cast<tFloat32>(M_PI)) -
            static_cast<tFloat32>(M_PI),
            mod(headingUpdate, 2.0f * static_cast<tFloat32>(M_PI)) - static_cast<tFloat32>(M_PI)));

    cv::Mat1d tmp = measurementMatrix * m_errorCov * measurementMatrix.t() + measCov;
    cv::Mat1d gain = m_errorCov * measurementMatrix.t() * tmp.inv();

    // update state and covariance matrix
    m_state += gain * innovation;
    m_state[2][0] = static_cast<double>(normalizeAngle(static_cast<tFloat32>(m_state[2][0]), 0.0f));
    m_errorCov = (identity - gain * measurementMatrix) * m_errorCov;
}

// wait for start-marker, and then initialize the filter
void MarkerPositioningFilter::initViaStartMarker(const tRoadSignDetectionResult &sign, const tFloat32 &d0,
                                                 const tFloat32 &a0) {
    for (auto &initialSign : initalRoadSigns) {
        if (initialSign.sign.id == sign.id && initialSign.sign.init) {
            // calculate the vehicle position and heading based on road-sign measurement
            const auto heading = normalizeAngle(initialSign.sign.heading, 0.0f);
            const auto correction = normalizeAngle(heading + a0 * DEG2RAD, 0.0f);
            // initialize filter state
            m_state[0][0] = initialSign.sign.pos.x + cos(correction) * -d0;
            m_state[1][0] = initialSign.sign.pos.y + sin(correction) * -d0;
            m_state[2][0] = heading;
            m_state[3][0] = 0;
            m_state[4][0] = 0;
            m_state[5][0] = 0;
            m_isInitialized = tTrue;
#ifdef DEBUG_MARKER_POS_LOG
            LOG_INFO(cString::Format("INIT %s (%f , %f)  Dir: %f Head: %f", mapTypeToString(sign.id).c_str(),
                                     initialSign.sign.pos.x, initialSign.sign.pos.y,
                                     initialSign.sign.heading * RAD2DEG, heading * RAD2DEG));
#endif
            //transmitUpdateTrafficSign(initialSign);
            break;
        }
    }
}

tResult MarkerPositioningFilter::ProcessIMU(IMediaSample *pMediaSampleIn) {
    IMU imu;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionInerMeasUnitData, pMediaSampleIn, pCoderInput);
        if (!imuIDs.m_bIDsInerMeasUnitSet) {
            pCoderInput->GetID("f32G_x", imuIDs.m_szIDInerMeasUnitF32G_x);
            pCoderInput->GetID("f32G_y", imuIDs.m_szIDInerMeasUnitF32G_y);
            pCoderInput->GetID("f32G_z", imuIDs.m_szIDInerMeasUnitF32G_z);
            pCoderInput->GetID("f32A_x", imuIDs.m_szIDInerMeasUnitF32A_x);
            pCoderInput->GetID("f32A_y", imuIDs.m_szIDInerMeasUnitF32A_y);
            pCoderInput->GetID("f32A_z", imuIDs.m_szIDInerMeasUnitF32A_z);
            pCoderInput->GetID("f32M_x", imuIDs.m_szIDInerMeasUnitF32M_x);
            pCoderInput->GetID("f32M_y", imuIDs.m_szIDInerMeasUnitF32M_y);
            pCoderInput->GetID("f32M_z", imuIDs.m_szIDInerMeasUnitF32M_z);
            pCoderInput->GetID("ui32ArduinoTimestamp", imuIDs.m_szIDInerMeasUnitArduinoTimestamp);
            imuIDs.m_bIDsInerMeasUnitSet = tTrue;
        }
        // pCoderInput->Get(imuIDs.m_szIDInerMeasUnitF32G_x, (tVoid *) &imu.G_x);
        // pCoderInput->Get(imuIDs.m_szIDInerMeasUnitF32G_y, (tVoid *) &imu.G_y);
        pCoderInput->Get(imuIDs.m_szIDInerMeasUnitF32G_z, (tVoid *) &imu.G_z);
        // pCoderInput->Get(imuIDs.m_szIDInerMeasUnitF32A_x, (tVoid *) &imu.A_x);
        // pCoderInput->Get(imuIDs.m_szIDInerMeasUnitF32A_y, (tVoid *) &imu.A_y);
        // pCoderInput->Get(imuIDs.m_szIDInerMeasUnitF32A_z, (tVoid *) &imu.A_z);
        // pCoderInput->Get(imuIDs.m_szIDInerMeasUnitF32M_x, (tVoid *) &imu.M_x);
        // pCoderInput->Get(imuIDs.m_szIDInerMeasUnitF32M_y, (tVoid *) &imu.M_y);
        // pCoderInput->Get(imuIDs.m_szIDInerMeasUnitF32M_z, (tVoid *) &imu.M_z);
        pCoderInput->Get(imuIDs.m_szIDInerMeasUnitArduinoTimestamp, (tVoid *) &imu.ArduinoTimestamp);
    }

    const auto dt = static_cast<tFloat64>(imu.ArduinoTimestamp - timeStamp) * 1e-6;
    timeStamp = imu.ArduinoTimestamp;

    if (!m_isInitialized) {
        RETURN_NOERROR;
    }
    // update heading
    auto hk = static_cast<tFloat32>(m_state[2][0]) +
              (imu.G_z * DEG2RAD + static_cast<tFloat32>(m_state[3][0])) * static_cast<tFloat32>(dt);
    // normalize heading -pi:pi
    hk = normalizeAngle(hk, 0.0f);

    // update speed and scale
    const auto ak = static_cast<tFloat32>(m_state[5][0]);
    const auto vk = speed * (speedScale - ak);

    // update transition matrix; F = I + Fc*dt
    m_transitionMatrix[0][2] = -vk * sin(hk) * dt;
    m_transitionMatrix[0][3] = -vk * sin(hk) * dt;
    m_transitionMatrix[0][4] = cos(hk) * dt;
    m_transitionMatrix[0][5] = -vk / (speedScale - ak) * cos(hk) * dt;

    m_transitionMatrix[1][2] = vk * cos(hk) * dt;
    m_transitionMatrix[1][3] = vk * cos(hk) * dt;
    m_transitionMatrix[1][4] = sin(hk) * dt;
    m_transitionMatrix[1][5] = -vk / (speedScale - ak) * sin(hk) * dt;

    m_transitionMatrix[2][3] = dt;

    // propagate state and covariance
    m_state[0][0] += vk * cos(hk) * dt;
    m_state[1][0] += vk * sin(hk) * dt;
    m_state[2][0] = hk;
    m_state[4][0] = vk;

    m_errorCov = m_transitionMatrix * m_errorCov * m_transitionMatrix.t() + m_processCov;

    Position pos;
    pos.pt.x = static_cast<float>(m_state[0][0]);
    pos.pt.y = static_cast<float>(m_state[1][0]);
    pos.radius = static_cast<float>(sqrt(m_errorCov[0][0]) + m_errorCov[1][1]);
    pos.heading = static_cast<float>(m_state[2][0]);
    pos.speed = static_cast<float>(m_state[4][0]);

    RETURN_IF_FAILED(sendPosition(GetTime(), pos));
    RETURN_IF_FAILED(checkForMissignSign(pos));
    RETURN_NOERROR;
}

tResult MarkerPositioningFilter::checkForMissignSign(const Position &pos) {
    if (thread.GetState() == cThread::TS_Suspended) {
        const float xd = pos.pt.x - oldPos.pt.x;
        const float yd = pos.pt.y - oldPos.pt.y;
        const float dist = sqrt(xd * xd + yd * yd);

        if (dist > minMovedDistance) {
            currentPos = pos;
            RETURN_IF_FAILED(thread.Run());
        }
    }
    RETURN_NOERROR;
}

tResult MarkerPositioningFilter::loadConfiguration() {
    initalRoadSigns.clear();
    cFilename fileRoadSigns = GetPropertyStr("Configuration");
    ADTF_GET_CONFIG_FILENAME(fileRoadSigns);
    fileRoadSigns = fileRoadSigns.CreateAbsolutePath(".");

    if (fileRoadSigns.IsEmpty()) {
        RETURN_ERROR(ERR_INVALID_FILE);
    }
    if (cFileSystem::Exists(fileRoadSigns)) {
        tinyxml2::XMLDocument xmlDocument;
        tinyxml2::XMLError eResult = xmlDocument.LoadFile(fileRoadSigns);
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

        for (tinyxml2::XMLElement *xmlRoadSignElement = pRoot->NextSibling()->FirstChildElement();
             xmlRoadSignElement != nullptr; xmlRoadSignElement = xmlRoadSignElement->NextSiblingElement()) {
            tinyxml2::XMLElement *tmpXmlRoadSignElement = xmlRoadSignElement;

            if (std::strcmp(tmpXmlRoadSignElement->Value(), "roadSign") == 0) {
                InitalRoadSignData marker;
                std::setlocale(LC_ALL, "en_US.UTF-8"); //floatingpoint workaround
                tmpXmlRoadSignElement->QueryIntAttribute("id", (int *) &marker.sign.id);
                tmpXmlRoadSignElement->QueryFloatAttribute("x", &marker.sign.pos.x);
                tmpXmlRoadSignElement->QueryFloatAttribute("y", &marker.sign.pos.y);
                tmpXmlRoadSignElement->QueryFloatAttribute("radius", &marker.sign.radius);
                tmpXmlRoadSignElement->QueryFloatAttribute("direction", &marker.sign.heading);
                tmpXmlRoadSignElement->QueryBoolAttribute("init", &marker.sign.init);
                std::setlocale(LC_ALL, "de_DE.UTF-8");
                marker.sign.heading *= DEG2RAD; // convert to radians
                if (marker.sign.init) {
                    marker.count = MARKER_SEEN; // Init Marker must be seeable
                } else {
                    marker.count = MARKER_NOT_SEEN;
                }
                marker.ticks = 0;
                initalRoadSigns.push_back(marker);
            }
        }
    } else {
        RETURN_ERROR(ERR_INVALID_FILE);
    }
    RETURN_NOERROR;
}

tResult MarkerPositioningFilter::ReadProperties(const tChar *strPropertyName) {
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MP_PROP_CAMERA_OFFSET_LAT)) {
        cameraOffsetLat = static_cast<float>(GetPropertyFloat(MP_PROP_CAMERA_OFFSET_LAT));
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MP_PROP_CAMERA_OFFSET_LON)) {
        cameraOffsetLon = static_cast<float>(GetPropertyFloat(MP_PROP_CAMERA_OFFSET_LON));
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MP_PROP_SPEED_SCALE)) {
        speedScale = static_cast<float>(GetPropertyFloat(MP_PROP_SPEED_SCALE));
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROPERTY_MAX_ANGLE)) {
        maxAngle = static_cast<float>(GetPropertyFloat(PROPERTY_MAX_ANGLE) * DEG2RAD);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROPERTY_MIN_MOV_DIST)) {
        minMovedDistance = static_cast<float>(GetPropertyFloat(PROPERTY_MIN_MOV_DIST));
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROPERTY_CHECK_OFFSET_X)
        || cString::IsEqual(strPropertyName, PROPERTY_CHECK_OFFSET_Y)) {
        checkMissignSignOffset = cv::Point2f(static_cast<float>(GetPropertyFloat(PROPERTY_CHECK_OFFSET_X)),
                                             static_cast<float>(GetPropertyFloat(PROPERTY_CHECK_OFFSET_Y)));
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROPERTY_RADIUS_MISSING_SIGN)) {
        checkRadius = static_cast<float>(GetPropertyFloat(PROPERTY_RADIUS_MISSING_SIGN));
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROPERTY_NOISE_COUNT)) {
        noiseCountMax = GetPropertyInt(PROPERTY_NOISE_COUNT);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PROPERTY_MARKER_COUNT)) {
        markerCountRepositioning = GetPropertyInt(PROPERTY_MARKER_COUNT);
    }
    RETURN_NOERROR;
}

tResult MarkerPositioningFilter::PropertyChanged(const char *strProperty) {
    ReadProperties(strProperty);
    RETURN_NOERROR;
}

tTimeStamp MarkerPositioningFilter::GetTime() {
    return adtf_util::cHighResTimer::GetTime();
}

tFloat32 MarkerPositioningFilter::angleDiff(const tFloat32 &angle1, const tFloat32 &angle2) {
    //normalize then compute difference and normalize in [-pi pi]
    return normalizeAngle(normalizeAngle(angle2, static_cast<tFloat32>(M_PI)) -
                          normalizeAngle(angle1, static_cast<tFloat32>(M_PI)), 0);
}

tFloat32 MarkerPositioningFilter::normalizeAngle(const tFloat32 &alpha, const tFloat32 &center) {
    return mod(alpha - center + static_cast<tFloat32>(M_PI), 2.0f * static_cast<tFloat32>(M_PI)) + center -
           static_cast<tFloat32>(M_PI);
}

tFloat32 MarkerPositioningFilter::mod(const tFloat32 &x, const tFloat32 &y) {
    double r;
    double b_x;
    if (y == floor(y)) {
        return x - floor(x / y) * y;
    } else {
        r = x / y;
        if (r < 0.0f) {
            b_x = ceil(r - 0.5f);
        } else {
            b_x = floor(r + 0.5f);
        }
        if (fabs(r - b_x) <= 2.2204460492503131E-16f * fabs(r)) {
            return 0.0f;
        } else {
            return static_cast<tFloat32>((r - floor(r)) * y);
        }
    }
}

void MarkerPositioningFilter::transmitInitalRoadSigns() {
    if (!outputInitalRoadSignsPin.IsConnected()) {
        return;
    } else if (!initalRoadSigns.empty()) {
        vector<tInitialRoadSign> signs;
        for (auto &inital : initalRoadSigns) {
            signs.push_back(inital.sign);
        }
        cObjectPtr<IMediaSample> pOutput;
        if (IS_OK(AllocMediaSample(&pOutput))) {
            pOutput->Update(_clock->GetTime(), &signs.front(), sizeof(tInitialRoadSign) * signs.size(), 0);
            outputInitalRoadSignsPin.Transmit(pOutput);
        }
    }
}

tResult MarkerPositioningFilter::sendPosition(const tTimeStamp &timeOfFix, const Position &pos) {
    __synchronized_obj(m_oSendPositionCritSection);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescPosition->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {
        __adtf_sample_write_lock_mediadescription(m_pDescPosition, pMediaSample, pCoder);
        if (!positionIDS.positionIDsSet) {
            pCoder->GetID("f32x", positionIDS.m_szIDPositionF32X);
            pCoder->GetID("f32y", positionIDS.m_szIDPositionF32Y);
            pCoder->GetID("f32radius", positionIDS.m_szIDPositionF32Radius);
            pCoder->GetID("f32speed", positionIDS.m_szIDPositionF32Speed);
            pCoder->GetID("f32heading", positionIDS.m_szIDPositionF32Heading);
            positionIDS.positionIDsSet = tTrue;
        }
        pCoder->Set(positionIDS.m_szIDPositionF32X, (tVoid *) &pos.pt.x);
        pCoder->Set(positionIDS.m_szIDPositionF32Y, (tVoid *) &pos.pt.y);
        pCoder->Set(positionIDS.m_szIDPositionF32Radius, (tVoid *) &pos.radius);
        pCoder->Set(positionIDS.m_szIDPositionF32Speed, (tVoid *) &pos.speed);
        pCoder->Set(positionIDS.m_szIDPositionF32Heading, (tVoid *) &pos.heading);
        pMediaSample->SetTime(timeOfFix);
    }
    RETURN_IF_FAILED(outputPositionPin.Transmit(pMediaSample));
    RETURN_NOERROR;
}

void MarkerPositioningFilter::transmitUpdateTrafficSign(const InitalRoadSignData &sign) {
    const auto id = static_cast<tInt16>(sign.sign.id);
    const auto x = static_cast<tFloat32>(sign.sign.pos.x);
    const auto y = static_cast<tFloat32>(sign.sign.pos.y);
    const auto direction = static_cast<tFloat32>(sign.sign.heading) * RAD2DEG;

    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid **) &pMediaSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescTrafficSignUpdate->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    pMediaSample->AllocBuffer(nSize);
    {
        __adtf_sample_write_lock_mediadescription(m_pDescTrafficSignUpdate, pMediaSample, pCoder);
        if (!trafficSignUpdateIDs.traffigSignUpdateIDsSet) {
            pCoder->GetID("i16Identifier", trafficSignUpdateIDs.m_szIDi16Identifier);
            pCoder->GetID("f32x", trafficSignUpdateIDs.m_szIDf32X);
            pCoder->GetID("f32y", trafficSignUpdateIDs.m_szIDf32Y);
            pCoder->GetID("f32angle", trafficSignUpdateIDs.m_szIDf32Angle);
            trafficSignUpdateIDs.traffigSignUpdateIDsSet = tTrue;
        }
        pCoder->Set(trafficSignUpdateIDs.m_szIDi16Identifier, (tVoid *) &id);
        pCoder->Set(trafficSignUpdateIDs.m_szIDf32X, (tVoid *) &x);
        pCoder->Set(trafficSignUpdateIDs.m_szIDf32Y, (tVoid *) &y);
        pCoder->Set(trafficSignUpdateIDs.m_szIDf32Angle, (tVoid *) &direction);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    outputChangedRoadSignPin.Transmit(pMediaSample);
}
