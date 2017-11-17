
#ifndef HTWK_WorldOutBaseFilter_H
#define HTWK_WorldOutBaseFilter_H

#include "stdafx.h"
#include <WorldService.h>
#include "../HTWK_Debug/EnableLogs.h"
class WorldOutBaseFilter : public adtf::cFilter {

private:
    cInputPin triggerInput;

    cObjectPtr<IMediaType> typeSignal;
    cObjectPtr<IMediaTypeDescription> descriptionSignal;

private:
    tResult CreateDescriptions(IException **__exception_ptr);

    tResult CreateInputPins(IException **__exception_ptr);

protected:
    cObjectPtr<WorldService> worldService;

protected:
    virtual tResult OnTrigger(tFloat32 interval) = 0;

public:
    WorldOutBaseFilter(const tChar *info);

    virtual ~WorldOutBaseFilter();

    virtual tResult Init(tInitStage eStage, __exception = nullptr);

    virtual tResult Shutdown(tInitStage eStage, IException **__exception_ptr);

    tResult OnPinEvent(IPin *sourcePin, tInt eventCode, tInt param1, tInt param2, IMediaSample *mediaSample);

};


#endif //HTWK_WorldOutBaseFilter_H
