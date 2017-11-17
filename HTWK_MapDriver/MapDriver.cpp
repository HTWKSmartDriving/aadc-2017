#include "MapDriver.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, MapDriver)

MapDriver::MapDriver(const tChar *__info) : cTimeTriggeredFilter(__info) {
    SetPropertyInt(INTERVAL_PROPERTY, INTERVAL_DEFAULT);
    SetPropertyStr(INTERVAL_PROPERTY
                           NSSUBPROP_DESCRIPTION, "Interval time in ms");
    SetPropertyInt(INTERVAL_PROPERTY
                           NSSUBPROP_MIN, 10);

    SetPropertyStr(MAP_DATA_PROPERTY, "map.xml");
    SetPropertyBool(MAP_DATA_PROPERTY
                            NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr(MAP_DATA_PROPERTY
                           NSSUBPROP_FILENAME
                           NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr(MAP_DATA_PROPERTY
                           NSSUBPROP_DESCRIPTION, "Map Data File to load");

    std::stringstream valueListStringBuilder;
    valueListStringBuilder << static_cast<int>(orientation::TurnDirection::LEFT) << "@" << "LEFT|"
                           << static_cast<int>(orientation::TurnDirection::STRAIGHT) << "@" << "STRAIGHT|"
                           << static_cast<int>(orientation::TurnDirection::RIGHT) << "@" << "RIGHT";

    SetPropertyInt(NEXT_TURN_PROPERTY, static_cast<int>(orientation::TurnDirection::STRAIGHT));
    SetPropertyBool(NEXT_TURN_PROPERTY NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(NEXT_TURN_PROPERTY NSSUBPROP_VALUELIST, valueListStringBuilder.str().c_str());
    SetPropertyStr(NEXT_TURN_PROPERTY NSSUBPROP_DESCRIPTION,
                   "Defines in which direction the Car should drive at the next crossing");
}

MapDriver::~MapDriver()
= default;

tResult MapDriver::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (StageFirst == eStage) {
        graphInitialized = tFalse;
        m_bIDsPositionSet = tFalse;
        m_bIDsSignalSet = tFalse;

        currentOrientation = 0;
        currentPosition = HTWKPoint();

        //read the track map from xml file
        RETURN_IF_FAILED(getMapFromFile());
        map.setTileLinks();

        RETURN_IF_FAILED(CreateDescriptions(__exception_ptr));
        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
    } else if (StageNormal == eStage) {
        auto interval = tUInt(GetPropertyInt(INTERVAL_PROPERTY));
        SetInterval(interval * 1000);

    } else if (StageGraphReady == eStage) {

    }

    RETURN_NOERROR;
}

tResult MapDriver::Shutdown(tInitStage eStage, __exception) {
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult MapDriver::CreateDescriptions(IException **__exception_ptr) {
    cObjectPtr<IMediaDescriptionManager> descManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                         (tVoid **) &descManager, __exception_ptr));

    //get SignalValue
    tChar const *signalValueDescription = descManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(signalValueDescription);
    signalValueStruct = new cMediaType(0, 0, 0, "tSignalValue", signalValueDescription,
                                       IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(signalValueStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &signalDescription));

    //get tPosition
    tChar const *tPositionDescription = descManager->GetMediaDescription("tPosition");
    RETURN_IF_POINTER_NULL(tPositionDescription);
    positionStruct = new cMediaType(0, 0, 0, "tPosition", tPositionDescription,
                                    IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(positionStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &positionDescription));

    RETURN_NOERROR;
}

tResult MapDriver::CreateInputPins(IException **__exception_ptr) {

    RETURN_IF_FAILED(positionPin.Create("position", new cMediaType(0, 0, 0, "tPosition"),
                                        static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&positionPin));
    RETURN_NOERROR;
}


tResult MapDriver::CreateOutputPins(IException **__exception_ptr) {

    RETURN_IF_FAILED(steeringPin.Create("steering", signalValueStruct, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&steeringPin));

    RETURN_NOERROR;
}

tResult MapDriver::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) {
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        // something was received
        RETURN_IF_POINTER_NULL(pMediaSample);

        if (pSource == &positionPin) {
            tFloat32 positionX, positionY, orientation;
            {
                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(positionDescription, pMediaSample, pCoder);

                // set the ids if not already done
                if (!m_bIDsPositionSet) {
                    pCoder->GetID("f32x", idPositionX);
                    pCoder->GetID("f32y", idPositionY);
                    pCoder->GetID("f32radius", idRadius);
                    pCoder->GetID("f32speed", idSpeed);
                    pCoder->GetID("f32heading", idHeading);

                    m_bIDsPositionSet = tTrue;
                }

                //get values from media sample
                pCoder->Get(idPositionX, (tVoid *) &positionX);
                pCoder->Get(idPositionY, (tVoid *) &positionY);
                pCoder->Get(idHeading, (tVoid *) &orientation);
            }

            currentPosition.setXval(positionX);
            currentPosition.setYval(positionY);
            currentOrientation = orientation;
            nextTurn = GetPropertyInt(NEXT_TURN_PROPERTY);
        }
    }

    RETURN_NOERROR;
}

tResult MapDriver::Cycle(IException **__exception_ptr) {

    if (!m_bIDsPositionSet) {
        RETURN_NOERROR;
    }

    trajectoryPoints.clear();

    if (!graphInitialized) {
        currentTilePoint = map.getTilePointByPosition(currentPosition, currentOrientation);
        graphInitialized = tTrue;
    }


    if (currentTilePoint->next[nextTurn] != nullptr) {
        if (HTWKUtils::Equals(currentPosition, *currentTilePoint->next[nextTurn],
                              0.3)) {
            currentTilePoint = currentTilePoint->next[nextTurn];
        }
    } else {
        if (HTWKUtils::Equals(currentPosition,
                              *currentTilePoint->next[static_cast<int>(orientation::TurnDirection::STRAIGHT)],
                              0.3)) {
            currentTilePoint = currentTilePoint->next[static_cast<int>(orientation::TurnDirection::STRAIGHT)];
        }
    }

    RETURN_IF_FAILED(calcTrackTrajectory());
    RETURN_IF_FAILED(findSteeringAndTransmit(currentPosition, currentOrientation * 180 / PI));

    RETURN_NOERROR;
}

tResult MapDriver::getMapFromFile() {
    // Get filename of map file
    cFilename fileMap = GetPropertyStr(MAP_DATA_PROPERTY);
    map.setFile(fileMap);
    RETURN_IF_FAILED(map.readXMLMap());

    RETURN_NOERROR;
}

tResult MapDriver::calcTrackTrajectory() {
    TilePoint *nextTilePoint;

    if (currentTilePoint->next[nextTurn] != nullptr) {
        map.getTrajectoryBetweenPoints(currentTilePoint,
                                       currentTilePoint->next[nextTurn],
                                       0.10, trajectoryPoints);

        nextTilePoint = currentTilePoint->next[nextTurn];

        if (nextTilePoint->next[static_cast<int>(orientation::TurnDirection::STRAIGHT)] != nullptr) {
            map.getTrajectoryBetweenPoints(nextTilePoint,
                                           nextTilePoint->next[static_cast<int>(orientation::TurnDirection::STRAIGHT)],
                                           0.10, trajectoryPoints);
        }

    } else {
        map.getTrajectoryBetweenPoints(currentTilePoint,
                                       currentTilePoint->next[static_cast<int>(orientation::TurnDirection::STRAIGHT)],
                                       0.10, trajectoryPoints);

        nextTilePoint = currentTilePoint->next[static_cast<int>(orientation::TurnDirection::STRAIGHT)];

        if (nextTilePoint->next[nextTurn] != nullptr) {
            map.getTrajectoryBetweenPoints(nextTilePoint,
                                           nextTilePoint->next[nextTurn],
                                           0.10, trajectoryPoints);
        } else {
            map.getTrajectoryBetweenPoints(nextTilePoint,
                                           nextTilePoint->next[static_cast<int>(orientation::TurnDirection::STRAIGHT)],
                                           0.10, trajectoryPoints);
        }
    }

    RETURN_NOERROR;

}

tResult MapDriver::findSteeringAndTransmit(const HTWKPoint &position, const tFloat32 &orientation) {
    CarTrajectory trajectoryCalculator;
    std::vector<HTWKPoint> carTrajecotryPoints;

    float steering = -30;
    std::pair<float, double> bestTrajectory(-30, DBL_MAX);

    while (steering < 30) {
        carTrajecotryPoints.clear();
        trajectoryCalculator.getTrajectoryPoints(steering, position, orientation, 0.10, carTrajecotryPoints);
        double score = trajectoryCalculator.getTrajectoryScore(carTrajecotryPoints, trajectoryPoints);

        //std::cout << "MapDriver Possible Steering: " << steering << " Score: " << score << std::endl;

        if (score < bestTrajectory.second) {
            bestTrajectory.first = steering;
            bestTrajectory.second = score;
        }

        steering += 1;
    }

    TransmitValue(steeringPin, bestTrajectory.first * (-100.0f / 30.0f));

    RETURN_NOERROR;
}

tResult MapDriver::TransmitValue(cOutputPin &pin, const tFloat32 value) {
    if (!pin.IsConnected())
    {
        RETURN_NOERROR;
    }

    cObjectPtr<IMediaSample> mediaSample;
    AllocMediaSample((tVoid **) &mediaSample);

    cObjectPtr<IMediaSerializer> serializer;
    signalDescription->GetMediaSampleSerializer(&serializer);
    mediaSample->AllocBuffer(serializer->GetDeserializedSize());

    tFloat32 f32Value = value;
    tUInt32 ui32TimeStamp = 0;

    {
        __adtf_sample_write_lock_mediadescription(signalDescription, mediaSample, pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid *) &f32Value);
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid *) &(ui32TimeStamp));
    }

    mediaSample->SetTime(_clock->GetStreamTime());
    pin.Transmit(mediaSample);

    RETURN_NOERROR;
}
