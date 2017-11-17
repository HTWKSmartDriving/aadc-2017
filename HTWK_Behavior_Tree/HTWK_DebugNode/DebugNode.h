#ifndef HTWK_DEBUGNODE_FILTER_H
#define HTWK_DEBUGNODE_FILTER_H

#include "stdafx.h"
//#include <aadc_structs.h>
#include "../../HTWK_Utils/HTWKUtils.hpp"
#include "../../HTWK_Types/BehaviorTreeData.h"

#define OID "htwk.bt.debug_node"
#define FILTER_NAME "HTWK BT - Debug Node"

class DebugNode : public adtf::cFilter {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter)

public:
    explicit DebugNode(const tChar *__info);

    ~DebugNode() override;

    tResult Init(tInitStage eStage, __exception) override;


protected:
    // static pins
    IMediaSample *inputMediaSample;
    cInputPin inputPin;
    cObjectPtr<IMediaType> mediaTypeIn;

    tResult CreateDescriptions(IException **__exception_ptr);

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

};

#endif // HTWK_DEBUGNODE_FILTER_H