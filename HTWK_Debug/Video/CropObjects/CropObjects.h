#ifndef HTWK_CROPOBJECTS_FILTER_H
#define HTWK_CROPOBJECTS_FILTER_H

#define OID_ADTF_FILTER_DEF "htwk.crop_objects"
#define ADTF_FILTER_DESC "HTWK Crop objects"

#include "stdafx.h"
#include <opencv2/core.hpp>
#include "../../../HTWK_Types/ObstacleData.h"

class CropObjects : public cFilter {
ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, OBJCAT_Auxiliary);

public:
    CropObjects(const tChar *__info);

    virtual ~CropObjects() override;

public:
    virtual tResult OnPinEvent(IPin *pSource, tInt nEventCode,
                               tInt nParam1, tInt nParam2,
                               IMediaSample *pMediaSample) override;

protected:
    virtual tResult Init(tInitStage eStage, __exception) override;

    virtual tResult Shutdown(tInitStage eStage, __exception) override;

    virtual tResult Start(__exception) override;

    virtual tResult Stop(__exception) override;

private:
    void Crop();

    cInputPin obstacleInputPin;
    std::vector<tObstacleData> obstacleData;
    tTimeStamp timeObstacle;
    cVideoPin imageInputPin;
    tBitmapFormat imageInputFormat;
    cv::Mat inputImage;
    tTimeStamp timeImage;

    long imageIdx;
    std::string strTargetPath;
    bool useHull;

};

#endif // HTWK_CROPOBJECTS_FILTER_H
