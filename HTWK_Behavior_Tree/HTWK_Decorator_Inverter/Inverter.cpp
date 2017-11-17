#include "Inverter.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, Inverter);

Inverter::Inverter(const tChar *__info) : DecoratorNode(__info) { }

Inverter::~Inverter() = default;

tResult Inverter::Init(tInitStage eStage, __exception) {
    // call parent implementation to set up static input and output pins
    RETURN_IF_FAILED(DecoratorNode::Init(eStage, __exception_ptr))

    RETURN_NOERROR;
}

tResult Inverter::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) {
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        RETURN_IF_POINTER_NULL(pMediaSample);
        if (pSource == &inputPin) {
            inputMediaSample = pMediaSample;
            BT::moduleState state;
            {
                // this will aquire the read lock on the sample and declare and initialize a pointer to the data
                __sample_read_lock(pMediaSample, BT::tStateSignal, pData);
                // now we can access the sample data through the pointer
                state = pData->state;
            }

            TransmitStatus(invert(state));
        }
    }

    RETURN_NOERROR;
}

tResult Inverter::TransmitStatus(BT::moduleState status) {
    BT::tStateSignal transmittingValue = {status, 0};
    cObjectPtr<IMediaSample> resultData;
    if (IS_OK(AllocMediaSample(&resultData))) {
        resultData->Update(inputMediaSample->GetTime(), &transmittingValue, sizeof(BT::tStateSignal), 0);
        outputPin.Transmit(resultData);
    }

    RETURN_NOERROR;
}

BT::moduleState Inverter::invert(BT::moduleState state) {
    if (state == BT::SUCCESS) {
        state = BT::FAIL;
    } else if (state == BT::FAIL) {
        state = BT::SUCCESS;
    }

    return state;
}



