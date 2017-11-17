#include "HTWKControllerPID.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID_HTWK_PIDCONTROLLER, HTWKControllerPID)

HTWKControllerPID::HTWKControllerPID(const tChar *__info) : cFilter(__info), m_f32LastMeasuredError(0),
                                                            m_f32SetPoint(0), m_lastSampleTime(0) {
    // Create the filter properties
    SetPropertyFloat("Controller Kp value", 1);
    SetPropertyFloat("Controller Ki value", 1);
    SetPropertyFloat("Controller Kd value", 1);

    SetPropertyInt("Sample Interval [msec]", 1);
    SetPropertyBool("use automatically calculated sample interval", 1);
    SetPropertyInt("Controller Typ", 1);
    SetPropertyStr("Controller Typ" NSSUBPROP_VALUELISTNOEDIT, "1@P|2@PI|3@PID");

    SetPropertyFloat("Upper Output Limit", 100);
    SetPropertyFloat("Lower Output Limit", -100);

}

HTWKControllerPID::~HTWKControllerPID() {
}

tResult HTWKControllerPID::CreateInputPins(__exception) {
    // create the input pins
    RETURN_IF_FAILED(m_oInputMeasured.Create("measured variable", new cMediaType(0, 0, 0, "tSignalValue"),
                                             static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputMeasured));
    RETURN_IF_FAILED(m_oInputSetPoint.Create("set point", new cMediaType(0, 0, 0, "tSignalValue"),
                                             static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputSetPoint));
    RETURN_NOERROR;
}

tResult HTWKControllerPID::CreateOutputPins(__exception) {
    //get the media description manager for this filter
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                         (tVoid **) &pDescManager, __exception_ptr));

    //get description for output pin
    tChar const *strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");

    // checks if exists
    RETURN_IF_POINTER_NULL(strDescSignalValue);

    //get mediatype for  data
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,
                                                             IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    //get mediatype description for arduino data type
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &m_pDescriptionSignal));

    // creates the output pin
    RETURN_IF_FAILED(
            m_oOutputManipulated.Create("manipulated variable", pTypeSignalValue, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputManipulated));
    RETURN_NOERROR;
}

tResult HTWKControllerPID::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst) {
        // create the pins
        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
    } else if (eStage == StageNormal) {

    } else if (eStage == StageGraphReady) {
        // no ids were set so far
        m_bIDsSignalSet = tFalse;
    }

    RETURN_NOERROR;
}

tResult HTWKControllerPID::Start(__exception) {
    return cFilter::Start(__exception_ptr);
}

tResult HTWKControllerPID::Stop(__exception) {
    return cFilter::Stop(__exception_ptr);
}

tResult HTWKControllerPID::Shutdown(tInitStage eStage, __exception) {
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult
HTWKControllerPID::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) {
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != nullptr &&
        m_pDescriptionSignal != nullptr) {
        // something was received
        RETURN_IF_POINTER_NULL(pMediaSample);

        if (pSource == &m_oInputMeasured) {
            //cObjectPtr<IMediaCoder> pCoder;
            //RETURN_IF_FAILED(m_pDescriptionSignal->Lock(pMediaSample, &pCoder));

            //write values with zero
            tFloat32 f32Value = 0;
            tUInt32 ui32TimeStamp = 0;
            tFloat32 f32OutputData = 0;
            {
                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(m_pDescriptionSignal, pMediaSample, pCoder);

                // set the ids if not already done
                if (!m_bIDsSignalSet) {
                    pCoder->GetID("f32Value", m_szIDSignalF32Value);
                    pCoder->GetID("ui32ArduinoTimestamp", m_szIDSignalArduinoTimestamp);
                    m_bIDsSignalSet = tTrue;
                }

                //get values from media sample
                pCoder->Get(m_szIDSignalF32Value, (tVoid *) &f32Value);
            }
            //calculation
            m_f32MeasuredVariable = f32Value;
            f32OutputData = getControllerValue(f32Value);

            //create new media sample
            cObjectPtr<IMediaSample> pMediaSampleTransmit;
            AllocMediaSample((tVoid **) &pMediaSampleTransmit);

            //allocate memory with the size given by the descriptor
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pDescriptionSignal->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pMediaSampleTransmit->AllocBuffer(nSize);

            {
                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(m_pDescriptionSignal, pMediaSampleTransmit, pCoder);

                // set the ids if not already done
                if (!m_bIDsSignalSet) {
                    pCoder->GetID("f32Value", m_szIDSignalF32Value);
                    pCoder->GetID("ui32ArduinoTimestamp", m_szIDSignalArduinoTimestamp);
                    m_bIDsSignalSet = tTrue;
                }

                //get values from media sample
                pCoder->Set(m_szIDSignalF32Value, (tVoid *) &f32OutputData);
                pCoder->Set(m_szIDSignalArduinoTimestamp, (tVoid *) &ui32TimeStamp);
            }

            //transmit media sample over output pin
            RETURN_IF_FAILED(pMediaSampleTransmit->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oOutputManipulated.Transmit(pMediaSampleTransmit));

        } else if (pSource == &m_oInputSetPoint) {
            //write values with zero
            tFloat32 f32Value = 0;
            tUInt32 ui32TimeStamp = 0;
            {
                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(m_pDescriptionSignal, pMediaSample, pCoder);

                // set the ids if not already done
                if (!m_bIDsSignalSet) {
                    pCoder->GetID("f32Value", m_szIDSignalF32Value);
                    pCoder->GetID("ui32ArduinoTimestamp", m_szIDSignalArduinoTimestamp);
                    m_bIDsSignalSet = tTrue;
                }

                //get values from media sample
                pCoder->Get(m_szIDSignalF32Value, (tVoid *) &f32Value);
                pCoder->Get(m_szIDSignalArduinoTimestamp, (tVoid *) &ui32TimeStamp);
            }

            m_f32SetPoint = f32Value;

        } else
            RETURN_NOERROR;


    }
    RETURN_NOERROR;
}


const tFloat32 HTWKControllerPID::getControllerValue(const tFloat32 &f32MeasuredValue) {
    //calculate the sample time in milliseceonds if necessary
    tInt iSampleTime;
    tFloat32 result = 0;
    if (GetPropertyBool("use automatically calculated sample interval", 1) == tTrue)
        iSampleTime = tInt(GetTime() - m_lastSampleTime) * 1000;
    else
        iSampleTime = GetPropertyInt("Sample Intervall [msec]", 1);
    m_lastSampleTime = GetTime();

    //the three controller algorithms
    if (GetPropertyInt("Controller Typ", 1) == 1) {
        //P-Regler y = Kp * e
        result = tFloat32(GetPropertyFloat("Controller Kp value", 1) * (m_f32SetPoint - f32MeasuredValue));
    } else if (GetPropertyInt("Controller Typ", 1) == 2) //PI- Regler
    {
        //esum = esum + e
        //y = Kp * e + Ki * Ta * esum
        m_f32AccumulatedVariable += (m_f32SetPoint - f32MeasuredValue);
        result = tFloat32(GetPropertyFloat("Controller Kp value", 1) * (m_f32SetPoint - f32MeasuredValue) \
 + GetPropertyFloat("Controller Ki value", 1) * iSampleTime * m_f32AccumulatedVariable);
    } else if (GetPropertyInt("Controller Typ", 1) == 3) {
        //esum = esum + e
        //y = Kp * e + Ki * Ta * esum + Kd * (e � ealt)/Ta
        //ealt = e
        m_f32AccumulatedVariable += (m_f32SetPoint - f32MeasuredValue);
        result = tFloat32(GetPropertyFloat("Controller Kp value", 1) * (m_f32SetPoint - f32MeasuredValue) \
 + tFloat32(GetPropertyFloat("Controller Ki value", 1)) * iSampleTime * m_f32AccumulatedVariable) \
 + tFloat32(GetPropertyFloat("Controller Kd value", 1)) *
   ((m_f32SetPoint - f32MeasuredValue) - m_f32LastMeasuredError) / iSampleTime;
        m_f32LastMeasuredError = (m_f32SetPoint - f32MeasuredValue);
    } else
        return 0;

    auto PIDMaximumOutput = tFloat32(GetPropertyFloat("Upper Output Limit"));
    auto PIDMinimumOutput = tFloat32(GetPropertyFloat("Lower Output Limit"));

    // checking for minimum and maximum limits
    if (result > PIDMaximumOutput) {
        result = PIDMaximumOutput;
    } else if (result < PIDMinimumOutput) {
        result = PIDMinimumOutput;
    }

    return result;
}

tTimeStamp HTWKControllerPID::GetTime() {
    return (_clock != NULL) ? _clock->GetTime() : cSystem::GetTime();
}
