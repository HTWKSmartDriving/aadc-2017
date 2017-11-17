#ifndef HTWK_MARKER_POS_FILTER
#define HTWK_MARKER_POS_FILTER

#include <cmath>
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <adtf_graphics.h>
#include <aadc_structs.h>
#include <opencv2/opencv.hpp>
#include "../HTWK_Utils/VectorHelper.h"
#include "../HTWK_Utils/ImageHelper.h"
#include <tinyxml2.h>
#include "../../HTWK_Types/RoadSignDetectionResult.h"
#include "../../HTWK_Types/InitalRoadSign.h"
#include "../../HTWK_Debug/EnableLogs.h"

using namespace adtf;
using namespace adtf_graphics;

#define MP_PROP_CAMERA_OFFSET_LAT "Camera Offset::Lateral"
#define MP_PROP_CAMERA_OFFSET_LON "Camera Offset::Longitudinal"
#define MP_PROP_SPEED_SCALE "Speed Scale"
// road sign distance and pose estimation
#define MP_LIMIT_ALPHA    60.0 // [degrees]
#define MP_LIMIT_DISTANCE  2.5 // [m]

// process covariances
#define MP_PROCESS_X                    1e-3
#define MP_PROCESS_Y                    1e-3
#define MP_PROCESS_HEADING              3e-4
#define MP_PROCESS_HEADING_DRIFT        5e-8
#define MP_PROCESS_SPEED                2e-3
#define MP_PROCESS_SPEED_SCALE          1e-6

// initial covariance values
#define MP_PROCESS_INIT_X               10.0
#define MP_PROCESS_INIT_Y               10.0
#define MP_PROCESS_INIT_HEADING         0.55
#define MP_PROCESS_INIT_HEADING_DRIFT   0.25
#define MP_PROCESS_INIT_SPEED           1.0
#define MP_PROCESS_INIT_SPEED_SCALE     0.5

// measurement covariances
#define MP_MEASUREMENT_X                0.5
#define MP_MEASUREMENT_Y                0.5
#define MP_MEASUREMENT_HEADING          1.0 // [radians]
#define RAD2DEG static_cast<tFloat32>(180.0/M_PI)
#define DEG2RAD static_cast<tFloat32>(M_PI/180.0)

#define PROPERTY_MAX_ANGLE "Maximum Angle"
#define PROPERTY_DEFAULT_ANGLE 65.0
#define PROPERTY_MIN_MOV_DIST "Missing Sign: Minimum Distance Moved"
#define PROPERTY_DEFAULT_MIN_MOV_DIST 0.25
#define PROPERTY_CHECK_OFFSET_X "Missign Sign: Offset X"
#define PROPERTY_CHECK_OFFSET_Y "Missign Sign: Offset Y"
#define PROPERTY_DEFAULT_CHECK_OFFSET_X 0.1
#define PROPERTY_DEFAULT_CHECK_OFFSET_Y 0.7
#define PROPERTY_RADIUS_MISSING_SIGN "Missing Sign: Check Radius"
#define PROPERTY_DEFAULT_RADIUS_MISSING_SIGN 0.7
#define PROPERTY_NOISE_COUNT "On Init: Drop of Noisymarkers"
#define PROPERTY_DEFAULT_NOISE_COUNT 30
#define PROPERTY_MARKER_COUNT "Repositioning Count"
#define PROPERTY_DEFAULT_MARKER_COUNT 2

#define MARKER_NOT_SEEN -1
#define MARKER_SEEN 0

#define OID_ADTF_FILTER_DEF "htwk.marker_positioning"
#define ADTF_FILTER_DESC "HTWK Marker Positioning"

class MarkerPositioningFilter : public adtf::cFilter, IThreadFunc {
ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, adtf::OBJCAT_Auxiliary);

public:
    MarkerPositioningFilter(const tChar *__info);

    ~MarkerPositioningFilter() = default;

protected:
    tResult Init(tInitStage eStage, __exception = NULL);

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

    tResult PropertyChanged(const char *strProperty);

    tResult Start(ucom::IException **__exception_ptr);

    tResult Stop(ucom::IException **__exception_ptr);

    tResult Shutdown(tInitStage eStage, __exception);

private:
    cCriticalSection m_critSecOnPinEvent;
    cCriticalSection m_oSendPositionCritSection;

    cObjectPtr<IMediaTypeDescription> m_pDescPosition;
    struct PositionIDS {
        tBufferID m_szIDPositionF32X;
        tBufferID m_szIDPositionF32Y;
        tBufferID m_szIDPositionF32Radius;
        tBufferID m_szIDPositionF32Speed;
        tBufferID m_szIDPositionF32Heading;
        tBool positionIDsSet = false;
    } positionIDS;
    struct Position {
        cv::Point2f pt = cv::Point2f(0, 0);
        float radius = 0;
        float speed = 0;
        float heading = 0;
    };
    Position currentPos;
    Position oldPos;

    cObjectPtr<IMediaTypeDescription> m_pDescTrafficSignUpdate;
    struct TrafficSignUpdateIDS {
        tBufferID m_szIDi16Identifier;
        tBufferID m_szIDf32X;
        tBufferID m_szIDf32Y;
        tBufferID m_szIDf32Angle;
        tBool traffigSignUpdateIDsSet = false;
    } trafficSignUpdateIDs;

    cObjectPtr<IMediaTypeDescription> m_pDescriptionInerMeasUnitData;
    struct IMUIDS {
        tBufferID m_szIDInerMeasUnitF32G_x;
        tBufferID m_szIDInerMeasUnitF32G_y;
        tBufferID m_szIDInerMeasUnitF32G_z;

        tBufferID m_szIDInerMeasUnitF32A_x;
        tBufferID m_szIDInerMeasUnitF32A_y;
        tBufferID m_szIDInerMeasUnitF32A_z;

        tBufferID m_szIDInerMeasUnitF32M_x;
        tBufferID m_szIDInerMeasUnitF32M_y;
        tBufferID m_szIDInerMeasUnitF32M_z;
        tBufferID m_szIDInerMeasUnitArduinoTimestamp;
        tBool m_bIDsInerMeasUnitSet = false;
    } imuIDs;
    struct IMU {
        tFloat32 G_x = 0;
        tFloat32 G_y = 0;
        tFloat32 G_z = 0;
        tFloat32 A_x = 0;
        tFloat32 A_y = 0;
        tFloat32 A_z = 0;
        tFloat32 M_x = 0;
        tFloat32 M_y = 0;
        tFloat32 M_z = 0;
        tUInt32 ArduinoTimestamp = 0;
    };

    struct InitalRoadSignData {
        tInitialRoadSign sign;
        int count = 0;
        tTimeStamp ticks;/*! measurement ticks*/
    };

    bool imuInit = false;
    bool speedInit = false;

    tResult ThreadFunc(cThread *Thread, tVoid *data, tSize size);

    vector<InitalRoadSignData> initalRoadSigns;

    tResult CreateOutputPins(__exception = NULL);

    tResult CreateInputPins(__exception = NULL);

    inline tResult ProcessIMU(IMediaSample *pMediaSampleIn);

    inline tResult ProcessRoadSigns(const tRoadSignDetectionResult &sign);

    tResult ReadProperties(const tChar *strPropertyName);

    tResult loadConfiguration();

    inline tResult sendPosition(const tTimeStamp &timeOfFix, const Position &pos);

    inline tTimeStamp GetTime();

    inline tFloat32 mod(const tFloat32 &x, const tFloat32 &y);

    inline tFloat32 normalizeAngle(const tFloat32 &alpha, const tFloat32 &center);

    inline tFloat32 angleDiff(const tFloat32 &angle1, const tFloat32 &angle2);

    inline void initViaStartMarker(const tRoadSignDetectionResult &sign, const tFloat32 &d0, const tFloat32 &a0);

    inline void updateEKF(const tFloat32 &d0, const tFloat32 &a0, const int &ind);

    inline int
    findMatchingRoadSign(const tRoadSignDetectionResult &sign, const tFloat32 &d0, const tFloat32 &a0, tFloat64 &dt);

    inline void transmitInitalRoadSigns();

    inline void transmitUpdateTrafficSign(const InitalRoadSignData &sign);

    cInputPin inputSpeedPin;

    cInputPin inputRoadSignsPin;

    cInputPin inputIMUPin;

    cOutputPin outputInitalRoadSignsPin;

    cOutputPin outputChangedRoadSignPin;

    cOutputPin outputPositionPin;

    cObjectPtr<IMediaTypeDescription> descMeasSpeed;
    tBufferID speedID;
    tBufferID speedArduinoTimestamp;
    tBool wheelSpeedIDsSet = false;
    tFloat32 speed = 0;
    tFloat32 speedScale = 1.0;

    tUInt32 timeStamp = 0;
    tFloat32 cameraOffsetLat = 0.0f;
    tFloat32 cameraOffsetLon = 0.3f;

    cv::Mat1d m_state; /*! filter state {X} */
    cv::Mat1d m_errorCov; /*! error covariance matrix {P} */
    cv::Mat1d m_processCov; /*! process covariance matrix {Q} */
    cv::Mat1d m_transitionMatrix; /*! state transition matrix {F} */
    tBool m_isInitialized; /*! initialization state of the filter */

    bool initalRoadSignsSend = false;
    float maxAngle = static_cast<float>(PROPERTY_DEFAULT_ANGLE * DEG2RAD);
    float checkRadius = PROPERTY_DEFAULT_RADIUS_MISSING_SIGN;
    float minMovedDistance = PROPERTY_DEFAULT_MIN_MOV_DIST;
    int noiseCountMax = PROPERTY_DEFAULT_NOISE_COUNT;
    int noiseCount = 0;
    int markerCountRepositioning = PROPERTY_DEFAULT_MARKER_COUNT;
    cv::Point2f checkMissignSignOffset = cv::Point2f(PROPERTY_DEFAULT_CHECK_OFFSET_X, PROPERTY_DEFAULT_CHECK_OFFSET_Y);
    cThread thread;

    inline tResult checkForMissignSign(const Position &pos);

    inline void checkChangedOrNewSign(const tRoadSignDetectionResult &sign, const int &index);
};

#endif
