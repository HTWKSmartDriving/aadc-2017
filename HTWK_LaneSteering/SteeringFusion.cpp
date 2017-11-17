#include "SteeringFusion.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, SteeringFusion)

SteeringFusion::SteeringFusion(const tChar *__info) : cFilter(__info) {
    lastRightLane = 0;
    lastLeftLane = 0;
    m_bIDsSignalSet = tFalse;
}

SteeringFusion::~SteeringFusion()
= default;

tResult SteeringFusion::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (StageFirst == eStage) {
        RETURN_IF_FAILED(CreateDescriptions(__exception_ptr));
        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
    } else if (StageNormal == eStage) {

    } else if (StageGraphReady == eStage) {
        _runtime->GetObject(OID_WORLD_SERVICE, IID_WORLD_INTERFACE, (tVoid **) &worldService, __exception_ptr);
    }

    RETURN_NOERROR;
}

tResult SteeringFusion::Shutdown(tInitStage eStage, __exception) {
    return cFilter::Shutdown(eStage, __exception_ptr);
}


tResult SteeringFusion::CreateDescriptions(IException **__exception_ptr) {
    cObjectPtr<IMediaDescriptionManager> descManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                         (tVoid **) &descManager, __exception_ptr));

    //get SignalValue
    tChar const *signalValueDescription = descManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(signalValueDescription);
    signalValueStruct = new cMediaType(0, 0, 0, "tSignalValue", signalValueDescription,
                                       IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(signalValueStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &signalDescription));

    RETURN_NOERROR;
}

tResult SteeringFusion::CreateInputPins(IException **__exception_ptr) {
    // create the input pins
    RETURN_IF_FAILED(laneLeftPin.Create("Left Lane offset", new cMediaType(0, 0, 0, "tSignalValue"),
                                        static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&laneLeftPin));
    RETURN_IF_FAILED(laneRightPin.Create("Right Lane offset", new cMediaType(0, 0, 0, "tSignalValue"),
                                         static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&laneRightPin));
    RETURN_NOERROR;
}

tResult
SteeringFusion::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) {
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != nullptr) {
        // something was received
        RETURN_IF_POINTER_NULL(pMediaSample);

        tFloat32 f32Value = 0;

        if (pSource == &laneLeftPin) {
            {
                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(signalDescription, pMediaSample, pCoder);

                // set the ids if not already done
                if (!m_bIDsSignalSet) {
                    pCoder->GetID("f32Value", m_szIDSignalF32Value);
                    pCoder->GetID("ui32ArduinoTimestamp", m_szIDSignalArduinoTimestamp);
                    m_bIDsSignalSet = tTrue;
                }

                //get values from media sample
                pCoder->Get(m_szIDSignalF32Value, (tVoid *) &f32Value);
            }

            lastLeftLane = f32Value;
        } else if (pSource == &laneRightPin) {
            {
                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(signalDescription, pMediaSample, pCoder);

                // set the ids if not already done
                if (!m_bIDsSignalSet) {
                    pCoder->GetID("f32Value", m_szIDSignalF32Value);
                    pCoder->GetID("ui32ArduinoTimestamp", m_szIDSignalArduinoTimestamp);
                    m_bIDsSignalSet = tTrue;
                }

                //get values from media sample
                pCoder->Get(m_szIDSignalF32Value, (tVoid *) &f32Value);
            }
            //
            lastRightLane = f32Value;

            //Steering of both lanes combined
            //Have to be inverted, because of pixel coordinate system
            tFloat32 steering = (lastLeftLane + lastRightLane) * (-1);
#ifdef DEBUG_STEERING_FUSION_DEBUG
            LOG_INFO(adtf_util::cString::Format("LaneSteering %f", steering));
#endif

            tSignalValue steeringStruct{static_cast<tUInt32>(_clock->GetStreamTime()), steering};
            worldService->Push(CAR_STEERING, steeringStruct);
        }
    }

    RETURN_NOERROR;
}