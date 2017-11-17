#ifndef HTWK_BT_SELECTOR_H
#define HTWK_BT_SELECTOR_H

#include "stdafx.h"
#include "../HTWK_CompositeNode/CompositeNode.h"
#include "../../HTWK_Types/BehaviorTreeData.h"
#include <iostream>
#include "../../HTWK_Debug/EnableLogs.h"
#define OID "htwk.bt_selector_filter"
#define FILTER_NAME "HTWK BT - Composite - Selector"
#define DYNAMIC_INPUT_PIN_NAME "d_input"

class Selector : public CompositeNode {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter);

private:

public:
    Selector(const tChar *__info);

    virtual ~Selector();

    tResult Init(tInitStage eStage, __exception) override;

private:
    tResult OnSuccess(IMediaSample *pMediaSample, unsigned int &actualPinNumber, BT::tStateSignal &receivedData);
    tResult OnFail(IMediaSample *pMediaSample, unsigned int &actualPinNumber, BT::tStateSignal &receivedData);
    tResult OnRunning(IMediaSample *pMediaSample, unsigned int &actualPinNumber, BT::tStateSignal &receivedData);

};

#endif //HTWK_BT_SELECTOR_H