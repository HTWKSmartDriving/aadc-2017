#ifndef MARKER_DETECT_FILTER_HEADER
#define MARKER_DETECT_FILTER_HEADER

#define OID_ADTF_FILTER_DEF "htwk.marker_detector"
#define ADTF_FILTER_DESC "HTWK Marker Detector"

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <adtf_graphics.h>
#include <adtf_utils.h>
#include "../../HTWK_Types/RoadSignDetectionResult.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "aruco_helpers.h"
#include <ImageHelper.h>

#define PROPERTY_DELAY "Delay"
#define PROPERTY_DEFAULT_DELAY 65

using namespace adtf;
using namespace adtf_graphics;
using namespace adtf_util;
using namespace cv;

class MarkerDetectorFilter : public cFilter, IThreadFunc {
ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, adtf::OBJCAT_Auxiliary);
public:
    MarkerDetectorFilter(const tChar *__info);

    virtual ~MarkerDetectorFilter();

    tResult Init(tInitStage eStage, ucom::IException **__exception_ptr);

    tResult
    OnPinEvent(adtf::IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, adtf::IMediaSample *pMediaSample);

    tResult Start(ucom::IException **__exception_ptr);

    tResult Stop(ucom::IException **__exception_ptr);

    tResult Shutdown(tInitStage eStage, __exception);

protected:
    cVideoPin inputVideoPin;

    cVideoPin outputDebugVideoPin;

    cOutputPin m_oPinRoadSignExt;

private:

    tTimeStamp previousTime = 0;
    tTimeStamp minTimeDifference = static_cast<tTimeStamp>(PROPERTY_DEFAULT_DELAY * 1e3); //TODO als Property
    tBitmapFormat inputBitmapFormat;

    tBitmapFormat outputBitmapFormat;

    /*! Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionRoadSignExt;
    /*! the id for the i16Identifier of the media description for output pin */
    tBufferID m_szIDRoadSignExtI16Identifier;
    /*! the id for the f32Imagesize of the media description for output pin */
    tBufferID m_szIDRoadSignExtF32Imagesize;
    /*! the id for the af32TVec of the media description for output pin */
    tBufferID m_szIDRoadSignExtAf32TVec;
    /*! the id for the af32RVec of the media description for output pin */
    tBufferID m_szIDRoadSignExtAf32RVec;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsRoadSignExtSet;

    tResult ThreadFunc(cThread *Thread, tVoid *data, tSize size);

    inline
    tResult sendRoadSignStructExt(const tInt16 &i16ID, const tFloat32 &f32MarkerSize, const tTimeStamp &timeOfFrame, const Vec3d &Tvec, const Vec3d &Rvec);

    Ptr<aruco::DetectorParameters> detectorParams;

    tFloat32 markerSize;

    Ptr<aruco::Dictionary> dictionary;

    cv::Mat intrinsic;

    cv::Mat distortion;

    cv::Mat inputImage;
    cv::Mat inputImageUnscaled;
    tBool useCrop;

    cThread thread;
};

#endif
