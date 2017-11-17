#include "DecoratorNode.h"

DecoratorNode::DecoratorNode(const tChar *__info) : cFilter(__info) {}

DecoratorNode::~DecoratorNode() = default;

tResult DecoratorNode::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {
        //this is a static Pin, creates Media description and output pin
        RETURN_IF_FAILED(CreateDescriptions(__exception_ptr));

        // create all static pins
        RETURN_IF_FAILED(inputPin.Create("input", mediaType, this));
        RETURN_IF_FAILED(RegisterPin(&inputPin));

        RETURN_IF_FAILED(outputPin.Create("output", mediaType, this));
        RETURN_IF_FAILED(RegisterPin(&outputPin));
    }

    RETURN_NOERROR;
}

tResult DecoratorNode::CreateDescriptions(IException **__exception_ptr) {
    // Fetch description manager
    cObjectPtr <IMediaDescriptionManager> descManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid * *) & descManager, __exception_ptr));

    const tChar *mediaDesc = descManager->GetMediaDescription("tStateSignal");
    RETURN_IF_POINTER_NULL(mediaDesc);

    mediaType = new cMediaType(0, 0, 0, "tStateSignal", mediaDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(mediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid * *) & mediaDesc));

    RETURN_NOERROR;
}
