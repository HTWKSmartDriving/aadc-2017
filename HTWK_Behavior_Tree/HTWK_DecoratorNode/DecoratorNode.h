#ifndef HTWK_DECORATOR_NODE_FILTER_H
#define HTWK_DECORATOR_NODE_FILTER_H

#include "stdafx.h"
#include "../../HTWK_Types/BehaviorTreeData.h"
#include "../../HTWK_Debug/EnableLogs.h"
#define DYNAMIC_OUTPUT_PIN_NAME "d_output"
#define DYNAMIC_INPUT_PIN_NAME "d_input"
#define PIN_COUNT_PROPERTY "pin_count"
#define PIN_COUNT_DEFAULT 0
#define PIN_COUNT_MIN 0
#define PIN_COUNT_MAX 100


class DecoratorNode : public adtf::cFilter {

public:
    explicit DecoratorNode(const tChar *__info);

    ~DecoratorNode() override;

    virtual tResult Init(tInitStage eStage, __exception);


protected:
    // static pins
    IMediaSample *inputMediaSample;
    cInputPin inputPin;
    cOutputPin outputPin;
    cObjectPtr<IMediaType> mediaType;

    tResult CreateDescriptions(IException **__exception_ptr);

};

#endif // HTWK_DECORATOR_NODE_FILTER_H

