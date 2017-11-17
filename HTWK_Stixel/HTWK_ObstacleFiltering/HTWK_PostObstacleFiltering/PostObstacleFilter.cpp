#include "PostObstacleFilter.h"

ADTF_FILTER_PLUGIN(OID_ADTF_FILTER_DEF, OID_ADTF_FILTER_DEF, PostObstacleFilter)

PostObstacleFilter::PostObstacleFilter(const tChar *__info) : cFilter(__info) {
    SetPropertyFloat(PROPERTY_MIN_DIST, PROPERTY_DEFAULT_MIN_DIST);
    SetPropertyBool(PROPERTY_MIN_DIST
                            NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PROPERTY_MIN_DIST
                           NSSUBPROP_DESCRIPTION,
                   "Minimum Distance in Metern to merge Obstacles to one, if Type is matching");
    SetPropertyFloat(PROPERTY_MIN_DIST
                             NSSUBPROP_MIN, 0);
    SetPropertyFloat(PROPERTY_MIN_DIST
                             NSSUBPROP_MAX, 1.0);

    SetPropertyFloat(PROPERTY_INTERVALL_CLEAN_UP_TIME, PROPERTY_DEFAULT_INTERVALL_CLEAN_UP_TIME);
    SetPropertyBool(PROPERTY_INTERVALL_CLEAN_UP_TIME
                            NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PROPERTY_INTERVALL_CLEAN_UP_TIME
                           NSSUBPROP_DESCRIPTION,
                   "If x seconds has passed, a time out check for tracked obstacles are done");
    SetPropertyFloat(PROPERTY_INTERVALL_CLEAN_UP_TIME
                             NSSUBPROP_MIN, 0);
    SetPropertyFloat(PROPERTY_INTERVALL_CLEAN_UP_TIME
                             NSSUBPROP_MAX, 60.0);

    SetPropertyFloat(PROPERTY_TRACKED_TIME_OUT, PROPERTY_DEFAULT_TRACKED_TIME_OUT);
    SetPropertyBool(PROPERTY_TRACKED_TIME_OUT
                            NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PROPERTY_TRACKED_TIME_OUT
                           NSSUBPROP_DESCRIPTION,
                   "If tracked obstacle wasn't seen more then x-seconds, it will be removed from tracking");
    SetPropertyFloat(PROPERTY_TRACKED_TIME_OUT
                             NSSUBPROP_MIN, 0);
    SetPropertyFloat(PROPERTY_TRACKED_TIME_OUT
                             NSSUBPROP_MAX, 60.0);

    SetPropertyInt(PROPERTY_MOVEMENT_COUNT, PROPERTY_DEFAULT_MOVEMENT_COUNT);
    SetPropertyBool(PROPERTY_MOVEMENT_COUNT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PROPERTY_MOVEMENT_COUNT NSSUBPROP_DESCRIPTION,
                   "If more then count overlapped, then this is seen ans moving");
    SetPropertyInt(PROPERTY_MOVEMENT_COUNT NSSUBPROP_MIN, 0);
    SetPropertyInt(PROPERTY_MOVEMENT_COUNT NSSUBPROP_MAX, 20);

    SetPropertyFloat(PROPERTY_THRESHOLD_BEST_SCORE, PROPERTY_DEFAULT_THRESHOLD_BEST_SCORE);
    SetPropertyBool(PROPERTY_THRESHOLD_BEST_SCORE NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PROPERTY_THRESHOLD_BEST_SCORE NSSUBPROP_DESCRIPTION, "Score equal or lower, will be dropped");
    SetPropertyFloat(PROPERTY_THRESHOLD_BEST_SCORE NSSUBPROP_MIN, 0);
    SetPropertyFloat(PROPERTY_THRESHOLD_BEST_SCORE NSSUBPROP_MAX, 1.0);
}

tResult PostObstacleFilter::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {
        cObjectPtr<IMediaType> pInputType;
        RETURN_IF_FAILED(AllocMediaType(&pInputType, MEDIA_TYPE_OBSTACLE_DATA, MEDIA_SUBTYPE_OBSTACLE_DATA,
                                        __exception_ptr));
        // create and register the input pin
        RETURN_IF_FAILED(obstacleInputPin.Create("Classified_Obstacles", pInputType, this));
        RETURN_IF_FAILED(RegisterPin(&obstacleInputPin));

        cObjectPtr<IMediaType> pOutputType;
        RETURN_IF_FAILED(
                AllocMediaType(&pOutputType, MEDIA_TYPE_TRACKING_DATA, MEDIA_SUBTYPE_TRACKING_DATA, __exception_ptr));
        RETURN_IF_FAILED(trackedOutputPin.Create("Tracked_Obstacles", pOutputType, this));
        RETURN_IF_FAILED(RegisterPin(&trackedOutputPin));


        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid **) &pDescManager, __exception_ptr));

        tChar const *strDescObstacle = pDescManager->GetMediaDescription("tObstacle");
        RETURN_IF_POINTER_NULL(strDescObstacle);
        cObjectPtr<IMediaType> pTypeObstacle = new cMediaType(0, 0, 0, "tObstacle", strDescObstacle,
                                                              IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(newObstacleJuryPin.Create("New_Obstacle_Jury", pTypeObstacle, this));
        RETURN_IF_FAILED(RegisterPin(&newObstacleJuryPin));
        RETURN_IF_FAILED(
                pTypeObstacle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &m_pDescNewObstacleJury));

        RETURN_IF_FAILED(
                videoDebugOutputPin.Create("Debug_Image", IPin::PD_Output, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&videoDebugOutputPin));

    } else if (eStage == StageGraphReady) {
    }
    RETURN_NOERROR;
}

tResult PostObstacleFilter::Start(ucom::IException **__exception_ptr) {
    trackedObstacles.clear();
    timeOutTrackedObstacles = static_cast<float>(GetPropertyFloat(PROPERTY_TRACKED_TIME_OUT));
    intervallCheckTime = static_cast<float>(GetPropertyFloat(PROPERTY_INTERVALL_CLEAN_UP_TIME));
    minimumMergeDistance = static_cast<float>(GetPropertyFloat(PROPERTY_MIN_DIST));
    countAsMovement = static_cast<int>(GetPropertyInt(PROPERTY_MOVEMENT_COUNT));
    thresholdBestScore = static_cast<float>(GetPropertyFloat(PROPERTY_THRESHOLD_BEST_SCORE));
    return cFilter::Start(__exception_ptr);
}

tResult PostObstacleFilter::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
                                       IMediaSample *pMediaSample) {
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        if (pSource == &obstacleInputPin) {
            vector<tObstacleData> newSamples;
            htwk::recieveVector(pMediaSample, newSamples);
            list<ObstacleTrackingData> matchedObstacles;
            if (findMatchingObstacles(newSamples, matchedObstacles)) {
                //Found Matching obstacles
                trackObstacles(matchedObstacles, pMediaSample->GetTime());
            }
        }
    }
    RETURN_NOERROR;
}

tResult PostObstacleFilter::transmitVideo(const Mat &image, cVideoPin &pin, tBitmapFormat &bmp) {
    updateOutputImageFormat(image, bmp, pin);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(bmp.nSize));
    RETURN_IF_FAILED(pMediaSample->Update(_clock->GetTime(), image.data, bmp.nSize, IMediaSample::MSF_None));
    RETURN_IF_FAILED(pin.Transmit(pMediaSample));
    RETURN_NOERROR;
}

bool PostObstacleFilter::findMatchingObstacles(vector<tObstacleData> &inputSamples,
                                               list<ObstacleTrackingData> &outputMatchedSamples) {
    //Kuer RearCam-Handling doesn't have an impact on "normal" tracking
    if (inputSamples.front().status == ObstacleStatus::REARCAM) {
        rearCamHandling(inputSamples);
        return false;
    }

    if (prevInputSamples.empty()) {
        prevInputSamples = move(inputSamples);
        currentTime = GetTime();
        return false;
    }

    for (auto &in : inputSamples) {
        for (auto &prev : prevInputSamples) {
            const auto rel = htwk::relationPoints::getRelationOf2Points(in.obstacle, prev.obstacle,
                                                                        static_cast<float>(in.radius),
                                                                        static_cast<float>(prev.radius));
            if (rel != htwk::relationPoints::NO_MATCH) {
                if (in.classification[0].score <= thresholdBestScore && in.classification[0].type != ObstacleType::PYLON) {
#ifdef DEBUG_POST_OBSTACLE_LOG
                    LOG_INFO("Dropped because of low Score");
#endif
                } else if (in.classification[0].type == ObstacleType::NONE ||
                           in.classification[0].type == ObstacleType::ROAD ||
                           in.classification[0].type == ObstacleType::SIGN) {
#ifdef DEBUG_POST_OBSTACLE_LOG
                    LOG_INFO("Dropped because Type is NONE, ROAD or Sign");
#endif
                } else {
                    ObstacleTrackingData data;
                    data.trackingData.radius = static_cast<float>(in.radius);
                    //Was an Pylon, so we ignore the threshold und use following type
                    if(in.classification[0].score <= thresholdBestScore){
                        data.trackingData.type = in.classification[1].type;
                    }else{
                        data.trackingData.type = in.classification[0].type;
                    }
                    //Init all with Static
                    data.trackingData.status = ObstacleStatus::STATIC;
                    data.trackingData.position = in.obstacle;
                    data.ticks = GetTime();
                    outputMatchedSamples.push_back(data);
                }
                break;
            }
        }
    }
    prevInputSamples.clear();
    prevInputSamples = move(inputSamples);
    return !outputMatchedSamples.empty();
}

void PostObstacleFilter::rearCamHandling(const vector<tObstacleData> &inputSamples) {
    for (auto &in : inputSamples) {
        if (in.classification[0].score > thresholdBestScore
            && in.classification[0].type == CAR) {
            ObstacleTrackingData data;
            data.trackingData.radius = 0;
            data.trackingData.type = CAR;
            //Init all with Static
            data.trackingData.status = ObstacleStatus::REARCAM;
            data.trackingData.position = in.obstacle;
            vector<tTrackingData> policeVecRearCam;
            policeVecRearCam.emplace_back(data.trackingData);
            transmitTrackedObstacles(policeVecRearCam, _clock->GetTime());
            //Found a police car behind us, so we transmit it
            return;
        }
    }
}

void PostObstacleFilter::trackObstacles(const list<ObstacleTrackingData> &matchedSamples, const tTimeStamp &time) {
    vector<tTrackingData> trackedObstaclesToTransmit;
    for (auto &match : matchedSamples) {
        bool isMatching = false;
        //holds best matching tracking iterator with minimumDistance
        pair<_List_iterator<ObstacleTrackingData>, float> bestMatchWithTrack
                = make_pair(trackedObstacles.end(), minimumMergeDistance);

        for (auto obst = trackedObstacles.begin(); obst != trackedObstacles.end(); ++obst) {
            float checkDistance = FLT_MAX;
            const auto rel = htwk::relationPoints::getRelationOf2Points(match.trackingData.position,
                                                                        obst->trackingData.position,
                                                                        match.trackingData.radius,
                                                                        obst->trackingData.radius, checkDistance);
            if (rel == htwk::relationPoints::NO_MATCH) {
                //Not matching but maybe in range from previous seen obstacle
                if (checkDistance < bestMatchWithTrack.second && obst->trackingData.type == match.trackingData.type) {
                    bestMatchWithTrack = make_pair(obst, checkDistance);
                }
                continue;
            }
            if (obst->trackingData.type == match.trackingData.type) {
                if (rel == htwk::relationPoints::OVERLAP) {
                    checkAndUpdateOverlap(match, obst);
                } else if (rel == htwk::relationPoints::FIRST_CONTAINS_SECOND ||
                           rel == htwk::relationPoints::SECOND_CONTAINS_FIRST) {
                    updateStaticObstaclePosition(match, obst);
                }
                trackedObstaclesToTransmit.emplace_back(obst->trackingData);
                isMatching = true;
                break;

            }
        }

        if (!isMatching) {
            //something near this position matches
            if (bestMatchWithTrack.first != trackedObstacles.end()) {
                checkAndUpdateOverlap(match, bestMatchWithTrack.first);
                bestMatchWithTrack.first->ticks = GetTime();
            } else {
                transmitNewObstaclesToJury(match.trackingData, time);
                trackedObstaclesToTransmit.emplace_back(match.trackingData);
                //transmitNewObstaclesToJury(match.trackingData, time);
                trackedObstacles.push_back(match);
            }
            trackedObstaclesToTransmit.emplace_back(match.trackingData);
        }
    }
    removeOutdatedObstacles();
    transmitTrackedObstacles(trackedObstaclesToTransmit, time);

#ifdef DEBUG_POST_OBSTACLE_LOG
    if (videoDebugOutputPin.IsConnected()) {
        debugWorldMap = Mat::zeros(Size(500, 500), CV_8UC3);
        for (const auto &item : trackedObstacles) {
            Point2f pt(item.trackingData.position.x * 50, 499 - item.trackingData.position.y * 50);
            if (!item.trackingData.newlyTracked) {
                circle(debugWorldMap, pt, item.trackingData.radius * 50, Scalar(255, 0, 0));
            } else {
                circle(debugWorldMap, pt, item.trackingData.radius * 50, Scalar(255, 255, 255));
            }
        }
        transmitVideo(debugWorldMap, videoDebugOutputPin, videoOutputBitmapFormat);
    }
#endif
}

void PostObstacleFilter::checkAndUpdateOverlap(const ObstacleTrackingData &match,
                                               list<PostObstacleFilter::ObstacleTrackingData>::iterator &obst) {
    if (checkMovableType(obst->trackingData.type)) {
        updateDynamicObstaclePosition(match, obst);
    } else {
        updateStaticObstaclePosition(match, obst);
    }
}

void PostObstacleFilter::updateDynamicObstaclePosition(const ObstacleTrackingData &match,
                                                       list<PostObstacleFilter::ObstacleTrackingData>::iterator &obst) {
    obst->trackingData.position = match.trackingData.position;
    obst->trackingData.radius = max(obst->trackingData.radius, match.trackingData.radius);
    obst->countForMovement++;
    if (obst->countForMovement > countAsMovement) {
        obst->trackingData.status = DYNAMIC;
        obst->trackingData.newlyTracked = false;
    }
    obst->ticks = GetTime();
}

void PostObstacleFilter::updateStaticObstaclePosition(const PostObstacleFilter::ObstacleTrackingData &match,
                                                      const list<PostObstacleFilter::ObstacleTrackingData>::iterator &obst) {
    obst->trackingData.position = (obst->trackingData.position + match.trackingData.position) / 2.0;
    obst->trackingData.radius = max(obst->trackingData.radius, match.trackingData.radius);
    obst->ticks = GetTime();
}

void PostObstacleFilter::removeOutdatedObstacles() {
    tTimeStamp newCurrentTime = GetTime();
    //Check every x-second for cleanup
    if (!checkTimeOut(newCurrentTime, currentTime, intervallCheckTime)) {
        return;
    }
    //More then x second has passed, clean trackedObstacles if longer then x seconds not seen
    for (auto it = trackedObstacles.begin(); it != trackedObstacles.end(); ++it) {
        if (checkTimeOut(newCurrentTime, it->ticks, timeOutTrackedObstacles)) {
            const auto remove = it--;
            trackedObstacles.erase(remove);
        }
    }
    currentTime = newCurrentTime;
}

void
PostObstacleFilter::transmitTrackedObstacles(const vector<tTrackingData> &trackedObstacles, const tTimeStamp &time) {
    if (!trackedObstacles.empty() && trackedOutputPin.IsConnected()) {
        cObjectPtr<IMediaSample> pOutput;
        if (IS_OK(AllocMediaSample(&pOutput))) {
            pOutput->Update(time, &trackedObstacles.front(), sizeof(tTrackingData) * trackedObstacles.size(), 0);
            trackedOutputPin.Transmit(pOutput);
        }
    }
}

tResult PostObstacleFilter::transmitNewObstaclesToJury(const tTrackingData &newObstacle, const tTimeStamp &time) {
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescNewObstacleJury->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {
        __adtf_sample_write_lock_mediadescription(m_pDescNewObstacleJury, pMediaSample, pCoder);
        if (!newObstacleIDs.newObstacleIDsSet) {
            pCoder->GetID("f32x", newObstacleIDs.m_szIDf32X);
            pCoder->GetID("f32y", newObstacleIDs.m_szIDf32Y);
            newObstacleIDs.newObstacleIDsSet = tTrue;
        }
        pCoder->Set(newObstacleIDs.m_szIDf32X, (tVoid *) &newObstacle.position.x);
        pCoder->Set(newObstacleIDs.m_szIDf32Y, (tVoid *) &newObstacle.position.y);

        pMediaSample->SetTime(time);
    }
    RETURN_IF_FAILED(newObstacleJuryPin.Transmit(pMediaSample));
    RETURN_NOERROR;
}

