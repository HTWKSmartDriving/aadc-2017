#include "DebugNode.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, DebugNode)

DebugNode::DebugNode(const tChar *__info) : cFilter(__info) {}

DebugNode::~DebugNode() = default;

tResult DebugNode::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {
        //this is a static Pin, creates Media description and output pin
        RETURN_IF_FAILED(CreateDescriptions(__exception_ptr));

        // create all static pins
        RETURN_IF_FAILED(inputPin.Create("input", mediaTypeIn, this));
        RETURN_IF_FAILED(RegisterPin(&inputPin));
    }

    RETURN_NOERROR;
}

tResult DebugNode::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) {

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        RETURN_IF_POINTER_NULL(pMediaSample);
        if (pSource == &inputPin) {
            inputMediaSample = pMediaSample;
            BT::moduleState state;
            {
                // this will aquire the read lock on the sample and declare and initialize a pointer to the data
                __sample_read_lock(pMediaSample, BT::tStateSignal , pData);
                // now we can access the sample data through the pointer
                state = pData->state;
            }

            if (state == BT::SUCCESS) {
                HTWKUtils::LogInfo("State - BT::SUCCESS");
            } else if (state == BT::FAIL) {
                HTWKUtils::LogInfo("State - BT::FAIL");
            } else if (state == BT::RUNNING) {
                HTWKUtils::LogInfo("State - BT::RUNNING");
            }
        }
    }

    RETURN_NOERROR;
}

tResult DebugNode::CreateDescriptions(IException **__exception_ptr) {
    cObjectPtr<IMediaDescriptionManager> descManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid **) &descManager, __exception_ptr));

    tChar const *mediaDescription = descManager->GetMediaDescription("BT::tStateSignal");
    RETURN_IF_POINTER_NULL(mediaDescription);

    mediaTypeIn = new cMediaType(0, 0, 0, "BT::tStateSignal", mediaDescription, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED( mediaTypeIn->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &mediaDescription));

    RETURN_NOERROR;
}