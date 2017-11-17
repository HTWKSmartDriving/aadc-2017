#ifndef HTWK_COMPOSITE_NODE_FILTER_H
#define HTWK_COMPOSITE_NODE_FILTER_H

#include "stdafx.h"
#include <aadc_structs.h>
#include "../../HTWK_Types/BehaviorTreeData.h"
#include "../HTWK_CompositeNode/CompositeNode.h"
#include "../../HTWK_Debug/EnableLogs.h"
#define DYNAMIC_OUTPUT_PIN_NAME "d_output"
#define DYNAMIC_INPUT_PIN_NAME "d_input"
#define PIN_COUNT_PROPERTY "pin_count"
#define PIN_COUNT_DEFAULT 0
#define PIN_COUNT_MIN 0
#define PIN_COUNT_MAX 100

class CompositeNode : public adtf::cFilter {

public:
    CompositeNode(const tChar *__info);
    virtual ~CompositeNode();

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);
    virtual tResult Init(tInitStage eStage, __exception = nullptr);

protected:
    int pinCount = 0;
    bool ignoreTimeTrigger = false;
    // normal Pins
    cInputPin triggerStaticInputPin;
    cOutputPin resultStaticOutputPin;
    // Pin Values
    cObjectPtr<IMediaType> resultStaticOutput;
    cObjectPtr<IMediaType> triggerStaticInput;
    // vectors of dynamic Pins
    std::vector<cObjectPtr<cDynamicInputPin> > dynamicInputPinList;
    std::vector<cObjectPtr<cDynamicOutputPin> > dynamicOutputPinList;

    tResult CreateDescriptions(IException **__exception_ptr);
    void sendResultAndFreeTimeTrigger(BT::tStateSignal &receivedData, IMediaSample *pMediaSample);
    virtual tResult OnSuccess(IMediaSample *pMediaSample, unsigned int &actualPinNumber, BT::tStateSignal &receivedData) = 0;
    virtual tResult OnFail(IMediaSample *pMediaSample, unsigned int &actualPinNumber, BT::tStateSignal &receivedData) = 0;
    virtual tResult OnRunning(IMediaSample *pMediaSample, unsigned int &actualPinNumber, BT::tStateSignal &receivedData) = 0;

    template<typename T1>
    void TransmitData(T1 &transmittingData, IPin &outputPin, IMediaSample *pMediaSample);
    template<typename T1>
    void TransmitData(T1 &transmittingData, cObjectPtr<cDynamicOutputPin> &outputPin,
                      IMediaSample *pMediaSample);


};


template<typename T1>
void CompositeNode::TransmitData(T1 &transmittingData, IPin &outputPin, IMediaSample *pMediaSample) {
    //transmit State to static output pin
    cObjectPtr<IMediaSample> resultData;
    if (IS_OK(AllocMediaSample(&resultData))) {
        resultData->Update(pMediaSample->GetTime(), &transmittingData, sizeof(T1), 0);
            outputPin.Transmit(resultData);
    }
}
template<typename T1>
void CompositeNode::TransmitData(T1 &transmittingData, cObjectPtr<cDynamicOutputPin> &outputPin, IMediaSample *pMediaSample) {
    //transmit State to static output pin
    cObjectPtr<IMediaSample> resultData;
    if (IS_OK(AllocMediaSample(&resultData))) {
        resultData->Update(pMediaSample->GetTime(), &transmittingData, sizeof(T1), 0);
            outputPin->Transmit(resultData);
    }
}

#endif // HTWK_COMPOSITE_NODE_FILTER_H

