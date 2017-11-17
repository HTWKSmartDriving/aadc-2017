#ifndef HTWK_IRTODISPARITY_FILTER
#define HTWK_IRTODISPARITY_FILTER

#define OID_ADTF_FILTER_DEF "htwk.ir_to_disparity"
#define ADTF_FILTER_DESC "HTWK IR to disparity"

#include "stdafx.h"
#include <opencv2/calib3d.hpp>
#include "../../../HTWK_Utils/ImageHelper.h"
#include "../../../HTWK_Types/CameraProperties.h"
#include "../../../HTWK_Debug/EnableLogs.h"
#define MAXIMUM_DISPARITY_DEFAULT 64
#define PROPERTY_MAX_DISP "Maximum Disparity"
#define FACTOR_16 16
#define FPS 30
#define WIDTH 640
#define HEIGHT 480

/**
 * IrToDisparity
 *
 * @author Philipp Kleinhenz
 */
class IrToDisparity : public cFilter {
    ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, OBJCAT_Converter);

public:
    IrToDisparity(const tChar* __info);
    virtual ~IrToDisparity() override;

public:
    virtual tResult OnPinEvent(IPin* pSource, tInt nEventCode,
                               tInt nParam1, tInt nParam2,
                               IMediaSample* pMediaSample) override;

protected:
    virtual tResult Init(tInitStage eStage, __exception) override;
    virtual tResult Shutdown(tInitStage eStage, __exception) override;
    virtual tResult Start(__exception) override;
    virtual tResult Stop(__exception) override;

private:
    tResult Process(IMediaSample* pData, IPin* pSource);
    tResult TransmitDisparity(const void* pData, const tTimeStamp &time);
    void initSGBM();

    cv::Ptr<cv::StereoSGBM> stereoSGBM;

    cVideoPin outputPinDisparity;
    tBitmapFormat bitmapFormatDisparity;
    int maxDisparity = MAXIMUM_DISPARITY_DEFAULT;

    cVideoPin irLeftInputPin;
    tTimeStamp timeLeft;
    cv::Mat irLeftInputImage;
    tBitmapFormat irLeftInputBitmapFormat;

    cVideoPin irRightInputPin;
    tTimeStamp timeRight;
    cv::Mat irRightInputImage;
    tBitmapFormat irRightInputBitmapFormat;

};

#endif // HTWK_IRTODISPARITY_FILTER
