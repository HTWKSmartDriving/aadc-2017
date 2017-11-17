#include "LaneSteeringFilter.h"

ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, LaneSteeringFilter);

LaneSteeringFilter::LaneSteeringFilter(const tChar *__info) : cFilter(__info) {
}

tResult LaneSteeringFilter::Init(tInitStage eStage, __exception) {
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst) {
        // get a media type for the input pin
        cObjectPtr<IMediaType> pInputType;
        RETURN_IF_FAILED(AllocMediaType(&pInputType,
                                        MEDIA_TYPE_CAMERA_RESOLUTION_DATA, MEDIA_SUBTYPE_CAMERA_RESOLUTION_DATA,
                                        __exception_ptr));
        // create and register the input pin
        RETURN_IF_FAILED(inputPinTypeCameraResolutionData.Create("Camera_Resolution_Data", pInputType, this));
        RETURN_IF_FAILED(RegisterPin(&inputPinTypeCameraResolutionData));

        pInputType = nullptr;
        RETURN_IF_FAILED(AllocMediaType(&pInputType, MEDIA_TYPE_LANE_DATA, MEDIA_SUBTYPE_LANE_DATA, __exception_ptr));
        // create and register the input pin
        RETURN_IF_FAILED(inputPinTypeLanePoint.Create("Lane_Detection_Result", pInputType, this));
        RETURN_IF_FAILED(RegisterPin(&inputPinTypeLanePoint));

        cObjectPtr<IMediaDescriptionManager> descManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid **) &descManager, __exception_ptr));

        //get SignalValue
        tChar const *signalValueDescription = descManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(signalValueDescription);
        signalValueStruct = new cMediaType(0, 0, 0, "tSignalValue", signalValueDescription,
                                           IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(signalValueStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &signalDescription));

        // create and regigster output pin
        RETURN_IF_FAILED(outputPinLeftLane.Create("Left_Lane", signalValueStruct, this));
        RETURN_IF_FAILED(RegisterPin(&outputPinLeftLane));

        RETURN_IF_FAILED(outputPinRightLane.Create("Right_Lane", signalValueStruct, this));
        RETURN_IF_FAILED(RegisterPin(&outputPinRightLane));

        RETURN_IF_FAILED(outputPinCenter.Create("Picture_Center", signalValueStruct, this));
        RETURN_IF_FAILED(RegisterPin(&outputPinCenter));

    }
    RETURN_NOERROR;
}

tResult LaneSteeringFilter::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
                                       IMediaSample *pMediaSample) {
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);
        if (pSource == &inputPinTypeCameraResolutionData) {
            {
                // this will aquire the read lock on the sample and declare and initialize a pointer to the data
                __sample_read_lock(pMediaSample, tCameraResolutionData, pData);
                // now we can access the sample data through the pointer
                tCameraResolutionData recievedSample = *pData;
                laneSteeringCalculator = unique_ptr<LaneSteeringCalculator>
                        (new LaneSteeringCalculator(recievedSample.width, recievedSample.height));
            }
        }

        if (pSource == &inputPinTypeLanePoint) {
            if (!laneSteeringCalculator) {
                LOG_ERROR(adtf_util::cString::Format("LaneSteeringCalculator was not initalized. Maybe missing CameraResolutionData!"));
                RETURN_NOERROR;
            }

            // this will store the value for our new sample
            vector<tLanePoint> tLanePointVector;
            {//------------------------------------------
                // this will aquire the read lock on the sample and declare and initialize a pointer to the data
                __sample_read_lock(pMediaSample, tLanePoint, pData);
                // now we can access the sample data through the pointer
                for(unsigned int i = 0; i < (pMediaSample->GetSize()/sizeof(tLanePoint)); i++){
                    tLanePointVector.push_back(pData[i]);
                }
            }//------------------------------------------

            // the read lock on the sample will be released when leaving this scope
            if (!tLanePointVector.empty()) {
                calcSteering(pMediaSample, tLanePointVector);
            }
        }
    }
    RETURN_NOERROR;
}

void LaneSteeringFilter::calcSteering(const IMediaSample *pMediaSample,
                                                 const vector<tLanePoint> &tLanePointVector) {
    laneSteeringCalculator->calcSteering(tLanePointVector);
    auto offsetLeft = laneSteeringCalculator->getLeftLaneOffset();
    auto offsetRight = laneSteeringCalculator->getRightLaneOffset();
    auto pictureCenter = laneSteeringCalculator->getPictureCenter().x();
    //LOG_INFO(adtf_util::cString::Format("LeftLaneOffset %f",offsetLeft));
    //LOG_INFO(adtf_util::cString::Format("RightLaneOffset %f",offsetRight));

    TransmitValue(outputPinLeftLane, offsetLeft);
    TransmitValue(outputPinRightLane, offsetRight);
    TransmitValue(outputPinCenter, pictureCenter);
}


tResult LaneSteeringFilter::TransmitValue(cOutputPin &pin, const double &value)
{
    if (!pin.IsConnected())
    {
        RETURN_NOERROR;
    }

    cObjectPtr<IMediaSample> mediaSample;
    AllocMediaSample((tVoid **) &mediaSample);

    cObjectPtr<IMediaSerializer> serializer;
    signalDescription->GetMediaSampleSerializer(&serializer);
    mediaSample->AllocBuffer(serializer->GetDeserializedSize());

    tFloat32 f32Value = static_cast<tFloat32>(value);
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
