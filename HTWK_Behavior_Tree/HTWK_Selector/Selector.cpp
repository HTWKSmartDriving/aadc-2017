#include "Selector.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, Selector);

Selector::Selector(const tChar *__info) : CompositeNode(__info) {}

Selector::~Selector()
= default;

tResult Selector::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(CompositeNode::Init(eStage, __exception_ptr));
    RETURN_NOERROR;
}

tResult Selector::OnSuccess(IMediaSample *pMediaSample, unsigned int &actualPinNumber, BT::tStateSignal &receivedData) {
#ifdef DEBUG_COMPOSITE_NOTE_LOG
    LOG_INFO(cString::Format("%s state is success", dynamicInputPinList.at(actualPinNumber)->GetName()));
#endif
    //return SUCCESS to static Output, dont trigger next dynamic Pin
    sendResultAndFreeTimeTrigger(receivedData, pMediaSample);
    RETURN_NOERROR;
}

tResult Selector::OnFail(IMediaSample *pMediaSample, unsigned int &actualPinNumber, BT::tStateSignal &receivedData) {
#ifdef DEBUG_COMPOSITE_NOTE_LOG
    LOG_INFO(cString::Format("%s state is fail", dynamicInputPinList.at(actualPinNumber)->GetName()));
#endif
    if (actualPinNumber < dynamicInputPinList.size() - 1) { //check if the actual pin, is not the last one of the config
        //trigger the next dynamic Pin
        tBoolSignalValue triggerValue;
        triggerValue.bValue = true;
        TransmitData<tBoolSignalValue>(triggerValue, dynamicOutputPinList.at(actualPinNumber + 1), pMediaSample);
    } else { //if actual pin is the laste one
        // return fail as result of Selector
        sendResultAndFreeTimeTrigger(receivedData, pMediaSample);
    }
    RETURN_NOERROR;
}

tResult Selector::OnRunning(IMediaSample *pMediaSample, unsigned int &actualPinNumber, BT::tStateSignal &receivedData) {
#ifdef DEBUG_COMPOSITE_NOTE_LOG
    LOG_INFO(cString::Format("%s state is running", dynamicInputPinList.at(actualPinNumber)->GetName()));
#endif
    sendResultAndFreeTimeTrigger(receivedData, pMediaSample);
    RETURN_NOERROR;
}