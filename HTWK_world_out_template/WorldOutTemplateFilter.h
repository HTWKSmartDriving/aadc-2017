#ifndef _WORLD_OUT_TEMPLATE_FILTER_
#define _WORLD_OUT_TEMPLATE_FILTER_

#include "stdafx.h"
#include <iostream>
#include <WorldOutBaseFilter.h>
#include "../HTWK_Types/ManeuverList.h"
#include <aadc_juryEnums.h>
#include <aadc_roadSign_enums.h>
#include <TrackMap.h>
#include "../../HTWK_Debug/EnableLogs.h"
#define OID "htwk.world_out_template_filter"
#define FILTER_NAME "HTWK World Out Template"

class WorldOutTemplateFilter : public WorldOutBaseFilter {
ADTF_FILTER(OID, FILTER_NAME, adtf::OBJCAT_DataFilter)

private:

public:
    WorldOutTemplateFilter(const tChar *__info);

    virtual ~WorldOutTemplateFilter();

    tResult Init(tInitStage eStage, __exception) override;

private:
    tResult Shutdown(tInitStage eStage, IException **__exception_ptr) override;

    tResult OnTrigger(tFloat32 interval);
};

#endif // _WORLD_OUT_TEMPLATE_FILTER_
