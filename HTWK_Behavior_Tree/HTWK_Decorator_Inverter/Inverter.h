#ifndef _DECORATOR_INVERTER_FILTER_H_
#define _DECORATOR_INVERTER_FILTER_H_

#include "../../HTWK_Utils/HTWKUtils.hpp"
#include "../HTWK_DecoratorNode/DecoratorNode.h"
#include "../../HTWK_Debug/EnableLogs.h"
#define OID "htwk.bt.decorator.inverter"
#define FILTER_NAME "HTWK BT - Decorator Inverter"

class Inverter : public DecoratorNode {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter);

public:
    explicit Inverter(const tChar *__info);

    ~Inverter() override;

protected:
    tResult Init(tInitStage eStage, ucom::IException **__exception_ptr) override;

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

    tResult TransmitStatus(BT::moduleState status);

private:
    BT::moduleState invert(BT::moduleState state);
};

#endif // _DECORATOR_INVERTER_FILTER_H_