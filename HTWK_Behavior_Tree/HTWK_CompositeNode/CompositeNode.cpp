#include <iostream>
#include "CompositeNode.h"

CompositeNode::CompositeNode(const tChar *__info) : cFilter(__info) {

    //using convenience method to configure dynamic config pins
    ConfigureConfigPins(tTrue, tTrue);

    // dynamic property example
    SetPropertyInt(PIN_COUNT_PROPERTY, PIN_COUNT_DEFAULT);
    SetPropertyInt(PIN_COUNT_PROPERTY NSSUBPROP_MINIMUM, PIN_COUNT_MIN);
    SetPropertyInt(PIN_COUNT_PROPERTY NSSUBPROP_MAXIMUM, PIN_COUNT_MAX);
    SetPropertyBool(PIN_COUNT_PROPERTY NSSUBPROP_REQUIRED, tTrue);

    // enable dynamic properties
    SetPropertyBool(NSPROP_CONFIGINFO_ENABLEDYNAMICPROPS, tTrue);
}

CompositeNode::~CompositeNode() = default;

tResult CompositeNode::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {
        // create all static pins
        //this is a static Pin, creates Media description and output pin
        RETURN_IF_FAILED(CreateDescriptions(__exception_ptr));
        RETURN_IF_FAILED(resultStaticOutputPin.Create("result",
                                                      resultStaticOutput,
                                                      this));
        RETURN_IF_FAILED(RegisterPin(&resultStaticOutputPin));

        //this is a static Pin, creates Media description and input pin
        RETURN_IF_FAILED(triggerStaticInputPin.Create("input",
                                                      triggerStaticInput,
                                                      this));
        RETURN_IF_FAILED(RegisterPin(&triggerStaticInputPin));

    } else if (eStage == StageNormal) {
        RETURN_IF_FAILED(CreateDescriptions(__exception_ptr));
        pinCount = GetPropertyInt(PIN_COUNT_PROPERTY);

        for (int i = 0; i < pinCount; i++) {

            //creates dynamically an ouput pin
            cString strPinName = std::string(DYNAMIC_OUTPUT_PIN_NAME + std::to_string(i)).c_str();
            if (strPinName.IsNotEmpty()) {
                cObjectPtr<cDynamicOutputPin> pDynamicOutputPin = new cDynamicOutputPin();

                if (IS_OK(pDynamicOutputPin->Create(strPinName,
                                                    triggerStaticInput,
                                                    static_cast<IPinEventSink *> (this)))) {
                    RegisterPin(pDynamicOutputPin);
                    dynamicOutputPinList.push_back(pDynamicOutputPin);

                } else {
                    THROW_ERROR_DESC(ERR_UNEXPECTED,
                                     cString::Format("Can not create the output pin \"%s\"!", strPinName.GetPtr()));
                }
            } else {
                THROW_ERROR_DESC(ERR_UNEXPECTED, "Can not create the output pin!");
            }

//          creates dynamically an input pin
            strPinName = std::string(DYNAMIC_INPUT_PIN_NAME + std::to_string(i)).c_str();

            if (strPinName.IsNotEmpty()) {
                cObjectPtr<cDynamicInputPin> pDynamicInputPin = new cDynamicInputPin();
                if (IS_OK(pDynamicInputPin->Create(strPinName,
                                                   resultStaticOutput,
                                                   static_cast<IPinEventSink *> (this)))) {
                    RegisterPin(pDynamicInputPin);
                    dynamicInputPinList.push_back(pDynamicInputPin);
                } else {
                    THROW_ERROR_DESC(ERR_UNEXPECTED,
                                     cString::Format("Can not create the output pin \"%s\"!", strPinName.GetPtr()));
                }
            } else {
                THROW_ERROR_DESC(ERR_UNEXPECTED, "Can not create the output pin!");
            }

        }
    }
    RETURN_NOERROR;
}


tResult
CompositeNode::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) {
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        RETURN_IF_POINTER_NULL(pMediaSample);
        if (pinCount > 0) { //check if dynamic Pins are set
            if (pSource == &triggerStaticInputPin) {
                if (!ignoreTimeTrigger) {
#ifdef DEBUG_COMPOSITE_NOTE_LOG
                    LOG_INFO("static");
#endif
                    tBoolSignalValue receivedTrigger;
                    {
                        // this will aquire the read lock on the sample and declare and initialize a pointer to the data
                        __sample_read_lock(pMediaSample, tBoolSignalValue, pData);
                        // now we can access the sample data through the pointer
                        receivedTrigger = *pData;
                    }
                    if (receivedTrigger.bValue) { //node is triggered
                        //trigger first dynamic Output
                        ignoreTimeTrigger = true;
                        TransmitData<tBoolSignalValue>(receivedTrigger, dynamicOutputPinList.at(0), pMediaSample);
                    }
                }
            }
            for (unsigned int pinNumber = 0; pinNumber < dynamicInputPinList.size(); pinNumber++) {
                if (pSource == dynamicInputPinList.at(pinNumber)) {
                    BT::tStateSignal receivedData;
                    {
                        // this will aquire the read lock on the sample and declare and initialize a pointer to the data
                        __sample_read_lock(pMediaSample, BT::tStateSignal, pData);
                        // now we can access the sample data through the pointer
                        receivedData = *pData;
                    }
                    if (receivedData.state == BT::SUCCESS) {
#ifdef DEBUG_COMPOSITE_NOTE_LOG
                        LOG_INFO(cString::Format("filtername: %s", OIGetInstanceName()));
#endif
                        OnSuccess(pMediaSample, pinNumber, receivedData);
                    } else if (receivedData.state == BT::FAIL) {
#ifdef DEBUG_COMPOSITE_NOTE_LOG
                        LOG_INFO(cString::Format("filtername: %s", OIGetInstanceName()));
#endif
                        OnFail(pMediaSample, pinNumber, receivedData);
                    } else if (receivedData.state == BT::RUNNING) {
#ifdef DEBUG_COMPOSITE_NOTE_LOG
                        LOG_INFO(cString::Format("filtername: %s", OIGetInstanceName()));
#endif
                        OnRunning(pMediaSample, pinNumber, receivedData);
#ifdef DEBUG_COMPOSITE_NOTE_LOG
                        LOG_INFO(cString::Format("ignore time trigger: %d", ignoreTimeTrigger));
#endif
                    }
                }
            }
        } else {
            RETURN_AND_LOG_ERROR_STR(ERR_NOT_CONNECTED, "no dynamic pins found");
        }
    }
    RETURN_NOERROR;
}

tResult CompositeNode::CreateDescriptions(IException **__exception_ptr) {
    cObjectPtr<IMediaDescriptionManager> descManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                         (tVoid **) &descManager, __exception_ptr));
    //create description for static pins
    //resultStaticOuputPin
    tChar const *resultOutputDescription = descManager->GetMediaDescription("tStateSignal");
    RETURN_IF_POINTER_NULL(resultOutputDescription);
    resultStaticOutput = new cMediaType(0, 0, 0, "tStateSignal", resultOutputDescription,
                                        IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(
            resultStaticOutput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &resultOutputDescription));


    resultOutputDescription = descManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(resultOutputDescription);
    triggerStaticInput = new cMediaType(0, 0, 0, "tBoolSignalValue", resultOutputDescription,
                                        IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(
            triggerStaticInput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &resultOutputDescription));
    RETURN_NOERROR;
}

void CompositeNode::sendResultAndFreeTimeTrigger(BT::tStateSignal &receivedData, IMediaSample *pMediaSample) {
    ignoreTimeTrigger = false;
    TransmitData<BT::tStateSignal>(receivedData, resultStaticOutputPin, pMediaSample);
}