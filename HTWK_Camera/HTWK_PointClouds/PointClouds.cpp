#include "PointClouds.h"

#include "../../HTWK_Utils/VectorHelper.h"
#include "../../HTWK_Utils/ImageHelper.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
// ###############################################################################################
#define CONSOLE_LOG(_text, _log_level) LOG_FN_OUTPUT((_text), _log_level)
#ifdef HTWK_POINTCLOUDS_DEBUG
    #define CONSOLE_LOG_INFO(_text)      CONSOLE_LOG(_text, adtf_util::LOG_LVL_INFO)
#endif // HTWK_POINTCLOUDS_DEBUG
#define CONSOLE_LOG_ERROR(_text)     CONSOLE_LOG(_text, adtf_util::LOG_LVL_ERROR)
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC,
                   OID_ADTF_FILTER_DEF,
                   PointClouds)
// ###############################################################################################
// Benchmarking helpers
#define HTWK_POINTCLOUDS_DEBUG_BENCH
#undef HTWK_POINTCLOUDS_DEBUG_BENCH

#ifdef HTWK_POINTCLOUDS_DEBUG_BENCH
    #include <chrono>
    typedef std::chrono::high_resolution_clock::time_point TimeVar;
    #define defTime() TimeVar t1
    #define _timeNow() std::chrono::high_resolution_clock::now()
    #define timeNow() t1 = _timeNow()
    #define duration(s) std::cout << s << std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(_timeNow() - t1).count()) << std::endl
#else
    #define defTime()
    #define duration(s)
    #define timeNow()
#endif // HTWK_POINTCLOUDS_DEBUG_BENCH
// ###############################################################################################
// Filter properties
#define PROP_INSTANT_START "Instant start"
#define PROP_FPS "FPS"
#define PROP_COLOR_MODE "Color mode"
#define PROP_DEPTH_MODE "Depth mode"
#define PROP_HULL "Convex hull"
static const char* PROP_S_INSTANT_START = PROP_INSTANT_START;
static const char* PROP_S_FPS = PROP_FPS;
static const char* PROP_S_COLOR_MODE = PROP_COLOR_MODE;
static const char* PROP_S_DEPTH_MODE = PROP_DEPTH_MODE;
static const int p_PROP_FPS_MIN = 1;
static const int p_PROP_FPS_MAX = 60;
static const char* PROP_S_HULL = PROP_HULL;
// ###############################################################################################
// Stream modes
/** valid color stream modes */
enum p_COLOR_MODES
{
    p_COLOR_320_240_30,
    p_COLOR_320_240_60,

    p_COLOR_640_480_30,
    p_COLOR_640_480_60,

    p_COLOR_1920_1080_15,
    p_COLOR_1920_1080_30,
};
/** valid depth stream modes */
enum p_DEPTH_MODES
{
    p_DEPTH_320_240_30,
    p_DEPTH_320_240_60,
    p_DEPTH_320_240_90,

    p_DEPTH_332_252_30,
    p_DEPTH_332_252_60,
    p_DEPTH_332_252_90,

    p_DEPTH_480_360_30,
    p_DEPTH_480_360_60,
    p_DEPTH_480_360_90,

    p_DEPTH_492_372_30,
    p_DEPTH_492_372_60,
    p_DEPTH_492_372_90,

    p_DEPTH_628_468_30,
    p_DEPTH_628_468_60,
    p_DEPTH_628_468_90,

    p_DEPTH_640_480_30,
    p_DEPTH_640_480_60,
    p_DEPTH_640_480_90
};
/** color stream mode parameters */
static const std::map<p_COLOR_MODES, PointClouds::tVideoMode> p_COLOR_MODE_MAP =
        {
                {p_COLOR_320_240_30, {320, 240, 30}},
                {p_COLOR_320_240_60, {320, 240, 60}},
                {p_COLOR_640_480_30, {640, 480, 30}},
                {p_COLOR_640_480_60, {640, 480, 60}},
                {p_COLOR_1920_1080_15, {1920, 1080, 15}},
                {p_COLOR_1920_1080_30, {1920, 1080, 30}}
        };
/** depth stream mode parameters */
static const std::map<p_DEPTH_MODES, PointClouds::tVideoMode> p_DEPTH_MODE_MAP =
        {
                {p_DEPTH_320_240_30, {320, 240, 30}},
                {p_DEPTH_320_240_60, {320, 240, 60}},
                {p_DEPTH_320_240_90, {320, 240, 90}},
                {p_DEPTH_332_252_30, {332, 252, 30}},
                {p_DEPTH_332_252_60, {332, 252, 60}},
                {p_DEPTH_332_252_90, {332, 252, 90}},
                {p_DEPTH_480_360_30, {480, 360, 30}},
                {p_DEPTH_480_360_60, {480, 360, 60}},
                {p_DEPTH_480_360_90, {480, 360, 90}},
                {p_DEPTH_492_372_30, {492, 372, 30}},
                {p_DEPTH_492_372_60, {492, 372, 60}},
                {p_DEPTH_492_372_90, {492, 372, 90}},
                {p_DEPTH_628_468_30, {628, 468, 30}},
                {p_DEPTH_628_468_60, {628, 468, 60}},
                {p_DEPTH_628_468_90, {628, 468, 90}},
                {p_DEPTH_640_480_30, {640, 480, 30}},
                {p_DEPTH_640_480_60, {640, 480, 60}},
                {p_DEPTH_640_480_90, {640, 480, 90}}
        };

// ###############################################################################################
/** micro seconds per second */
static const int p_MICRO_IN_SEC = 1000000;
// ###############################################################################################
// Parameters
// all parameters regarding length or distance are in meters!

// streams
/** default color stream mode */
static const p_COLOR_MODES p_DEFAULT_COLOR_MODE = p_COLOR_1920_1080_30;
/** default depth stream mode */
static const p_DEPTH_MODES p_DEFAULT_DEPTH_MODE = p_DEPTH_480_360_30;
/** default color stream */
static const rs::stream p_COLOR_STREAM = rs::stream::rectified_color;
/** default depth stream */
static const rs::stream p_DEPTH_STREAM = rs::stream::points;

// range filtering
// points outside of this range will be discarded
/** minimum point z value */
static const float p_Z_MIN = 0.600f;
/** maximum point z value */
static const float p_Z_MAX = 3.125f;
/** minimum point y value */
static const float p_Y_MIN = -0.25f;
/** maximum point y value */
static const float p_Y_MAX = 0.125f;

// subsampling
/** minimum leaf size for voxel grid subsampling */
static const float p_VOXEL_SUBSAMPLING_LEAF_SIZE_MIN = 0.03125f;

// finding the ground plane
/** only consider points with at most this Z coordinate for SAC */
static const float p_SAC_SPACE_Z_MAX = 1.0f;
/** only cosider points with at least this Z coordinate for SAC */
static const float p_SAC_SPACE_Z_MIN = p_Z_MIN;
/** terminate SAC algorithm if good model has not been found within this many iterations */
static const int p_SAC_MAX_ITERATIONS = 500;
/** count points within this distance from the plane as covered by the SAC model */
static const double p_SAC_PLANE_DISTANCE_MAX = 0.025;
/** restrict plane parameters to planes perpendicular to this axis for SAC */
static const Eigen::Vector3f p_SAC_PLANE_PERPENDICULAR_AXIS(0.0, 1.0, 0.0);
/** allow some rotational freedom of the SAC plane out of the axis restriction */
static const double p_SAC_PLANE_ROTATION_FREEDOM = 8.0;

// below ground plane filtering
/** remove points that are not further above the ground */
static const float p_REMOVE_POINTS_ABOVE_GROUND_DISTANCE = 0.02f;

// outlier filtering
/** minimum amount of neighbors each point has to have to not be discarded */
static const int p_INLIER_MIN_NEIGHBORS = 16;
/** search radius when searching for point neighbors */
static const double p_INLIER_SEARCH_RADIUS = 0.08;
/** minimun cluster size */
static const int p_CLUSTER_SIZE_MIN = 8;
/** maximum cluaster size */
static const int p_CLUSTER_SIZE_MAX = 310;

// clustering
/** maxmiumum point distance from the cluster before applying the clustering condition */
static const float p_CLUSTER_PRECONDITION_DISTANCE_MAX = 0.5f;
/** maximum point distance from the cluster when applying the cluster condition */
static const float p_CLUSTER_DISTANCE_MAX = 0.08f;
/** special distance if a point overlaps the cluster in the XY plane */
static const float p_CLUSTER_DISTANCE_MAX_FOR_XY_OVERLAP = 0.09f;
/** consider points not further apart on Y axis to be overlapping on the XY plane with the cluster */
static const float p_CLUSTER_DISTANCE_MAX_OVERLAP_MAX_DIST_Y = 0.05f;
/** consider points not further apart on X axis to be overlapping on the XY plane with the cluster */
static const float p_CLUSTER_DISTANCE_MAX_OVERLAP_MAX_DIST_X = 0.05f;

// bounding box constraints
/** maximum bounding box width */
static const float p_BBOX_WIDTH_MAX = 0.8f;
/** minimum bounding box width */
static const float p_BBOX_WIDTH_MIN = 0.03f;
/** maximum bounding box height */
static const float p_BBOX_HEIGHT_MAX = 0.5f;
/** minimum bounding box height */
static const float p_BBOX_HEIGHT_MIN = 0.04f;
/** minium bounding box XY area */
static const float p_BBOX_MIN_AREA = 0.0015f;
/** any bounding box that begins farther away from the ground (e.g. "floating" bbox) is discarded */
static const float p_BBOX_MAX_DISTANCE_TO_GROUND = 0.08f;
/** pad resulting bounding boxes on all axis by this amount */
static const float p_BBOX_PADDING = 0.01f;

// world coordiante mapping
/** translation vector from IMU to camera origin */
static const cv::Point2f p_RS_TO_IMU = cv::Point2f(0.04f, 0.363f);

// threading
/** microsecods to wait each thread loop iteration when waiting for the initial position */
static const int p_TIME_TO_WAIT_EACH_CALL_UNTIL_INITIAL_POSITION = 200000;


// ###############################################################################################
PointClouds::PointClouds(const tChar* __info) :
        cFilter(__info),
        modeColor(p_COLOR_MODE_MAP.at(p_DEFAULT_COLOR_MODE)),
        modeDepth(p_DEPTH_MODE_MAP.at(p_DEFAULT_DEPTH_MODE)),
        fps((modeColor.fps < modeDepth.fps) ? modeColor.fps : modeDepth.fps),
        isSyntheticFps(false),
        syntheticFrameTime(p_MICRO_IN_SEC/fps),
        instantStart(false),
        firstPositionReceived(false),
        obstacleId(0),
        makeHull(false),
        cameraDevice(nullptr),
        pointCloud(nullptr),
        pointCloudFiltered(nullptr),
        newBuffer(false)
#ifdef HTWK_POINTCLOUDS_DEBUG
       ,viewer(nullptr)
#endif // HTWK_POINTCLOUDS_DEBUG
{
    SetPropertyInt(PROP_FPS, fps);
    SetPropertyBool(PROP_FPS NSSUBPROP_REQUIRED, tTrue);
    SetPropertyInt(PROP_FPS NSSUBPROP_MIN, p_PROP_FPS_MIN);
    SetPropertyInt(PROP_FPS NSSUBPROP_MAX, p_PROP_FPS_MAX);
    SetPropertyStr(PROP_FPS NSSUBPROP_DESCRIPTION,
                   "Effective processing frame rate. "
                   "Values greater than the fps of either color or depth stream will not have any effect.");

    SetPropertyBool(PROP_INSTANT_START, tFalse);
    SetPropertyStr(PROP_INSTANT_START NSSUBPROP_DESCRIPTION,
                   "Start detection without waiting for first position information");

    std::stringstream valueListColorModes;
    valueListColorModes << p_COLOR_320_240_30 << "@" << "320x240_30|"
                        << p_COLOR_320_240_60 << "@" << "320x240_60|"
                        << p_COLOR_640_480_30 << "@" << "640x480_30|"
                        << p_COLOR_640_480_60 << "@" << "640x480_60|"
                        << p_COLOR_1920_1080_15 << "@" << "1920x1080_15|"
                        << p_COLOR_1920_1080_30 << "@" << "1920x1080_30";

    SetPropertyInt(PROP_COLOR_MODE, p_DEFAULT_COLOR_MODE);
    SetPropertyBool(PROP_COLOR_MODE NSSUBPROP_REQUIRED, tTrue);
    SetPropertyStr(PROP_COLOR_MODE NSSUBPROP_VALUELIST, valueListColorModes.str().c_str());
    SetPropertyStr(PROP_COLOR_MODE NSSUBPROP_DESCRIPTION,
                   "Color resolution and FPS");

    std::stringstream valueListDepthModes;
    valueListDepthModes << p_DEPTH_320_240_30 << "@" << "320x240_30|"
                        << p_DEPTH_320_240_60 << "@" << "320x240_60|"
                        << p_DEPTH_320_240_90 << "@" << "320x240_90|"
                        << p_DEPTH_332_252_30 << "@" << "332x252_30|"
                        << p_DEPTH_332_252_60 << "@" << "332x252_60|"
                        << p_DEPTH_332_252_90 << "@" << "332x252_90|"
                        << p_DEPTH_480_360_30 << "@" << "480x360_30|"
                        << p_DEPTH_480_360_60 << "@" << "480x360_60|"
                        << p_DEPTH_480_360_90 << "@" << "480x360_90|"
                        << p_DEPTH_492_372_30 << "@" << "492x372_30|"
                        << p_DEPTH_492_372_60 << "@" << "492x372_60|"
                        << p_DEPTH_492_372_90 << "@" << "492x372_90|"
                        << p_DEPTH_628_468_30 << "@" << "628x468_30|"
                        << p_DEPTH_628_468_60 << "@" << "628x468_60|"
                        << p_DEPTH_628_468_90 << "@" << "628x468_90|"
                        << p_DEPTH_640_480_30 << "@" << "640x480_30|"
                        << p_DEPTH_640_480_60 << "@" << "640x480_60|"
                        << p_DEPTH_640_480_90 << "@" << "640x480_90";

    SetPropertyInt(PROP_DEPTH_MODE, p_DEFAULT_DEPTH_MODE);
    SetPropertyBool(PROP_DEPTH_MODE NSSUBPROP_REQUIRED, tTrue);
    SetPropertyStr(PROP_DEPTH_MODE NSSUBPROP_VALUELIST, valueListDepthModes.str().c_str());
    SetPropertyStr(PROP_DEPTH_MODE NSSUBPROP_DESCRIPTION,
                   "Depth resolution and FPS");

    SetPropertyBool(PROP_HULL, tFalse);
    SetPropertyBool(PROP_HULL NSSUBPROP_REQUIRED, tTrue);
    SetPropertyStr(PROP_HULL NSSUBPROP_DESCRIPTION,
                    "Compute convex hull for detected obstacles");
}

// ###############################################################################################
PointClouds::~PointClouds()
{}

// ###############################################################################################
tResult PointClouds::PropertyChanged(const tChar* strName)
{
    if (PROP_S_INSTANT_START == strName)
    {
        instantStart = GetPropertyBool(PROP_INSTANT_START);
    }
    else if (PROP_S_FPS == strName)
    {
        fps = static_cast<unsigned int>(GetPropertyInt(PROP_FPS));
    }
    else if (PROP_S_COLOR_MODE == strName)
    {
        modeColor = p_COLOR_MODE_MAP.at(static_cast<p_COLOR_MODES>(GetPropertyInt(PROP_COLOR_MODE)));
    }
    else if (PROP_S_DEPTH_MODE == strName)
    {
        modeDepth = p_DEPTH_MODE_MAP.at(static_cast<p_DEPTH_MODES>(GetPropertyInt(PROP_DEPTH_MODE)));
    }
    else if (PROP_S_HULL == strName)
    {
        makeHull = GetPropertyBool(PROP_HULL);
    }

    return cFilter::PropertyChanged(strName);
}

// ###############################################################################################
tResult PointClouds::Start(__exception)
{
#ifdef HTWK_POINTCLOUDS_DEBUG
    viewer.reset(new pcl::visualization::CloudViewer("cloud"));
#endif // HTWK_POINTCLOUDS_DEBUG
    if (nullptr != cameraDevice)
    {
        cameraDevice->start();
        // start the thread if not running by now
        if (thread.GetState() != cKernelThread::TS_Running)
        {
            thread.Run(tTrue);
        }
    }else{
        THROW_ERROR_DESC(ERR_NOT_CONNECTED, "PointClouds::RealsenseCamera not available!");
    }
    return cFilter::Start(__exception_ptr);
}

// ###############################################################################################
tResult PointClouds::Stop(__exception)
{
    //suspend the thread
    if (thread.GetState() == cKernelThread::TS_Running)
    {
        thread.Suspend(tTrue);
    }
    if (nullptr != cameraDevice)
    {
        cameraDevice->stop();
    }
#ifdef HTWK_POINTCLOUDS_DEBUG
    viewer.reset();
#endif // HTWK_POINTCLOUDS_DEBUG
    return cFilter::Stop(__exception_ptr);
}

// ###############################################################################################
tResult PointClouds::Shutdown(cFilter::tInitStage eStage, __exception)
{
    // release the thread
    if (StageNormal == eStage)
    {
        cameraMutex.Enter();
        thread.Terminate(tTrue);
        thread.Release();
        cameraMutex.Leave();
    }
    return cFilter::Shutdown(eStage, __exception_ptr);
}

// ###############################################################################################
tResult PointClouds::Init(cFilter::tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    // update all property values
    PropertyChanged(PROP_S_COLOR_MODE);
    PropertyChanged(PROP_S_DEPTH_MODE);
    PropertyChanged(PROP_S_FPS);
    PropertyChanged(PROP_S_INSTANT_START);
    PropertyChanged(PROP_S_HULL);

    if (eStage == StageFirst)
    {
        // set up pins

        // rgb image output
        RETURN_IF_FAILED(outputPinRgb.Create("RGB", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&outputPinRgb));

        // obstacle data output
        cObjectPtr<IMediaType> pOutputType;
        pOutputType = NULL;
        RETURN_IF_FAILED(AllocMediaType(&pOutputType,
                                        MEDIA_TYPE_OBSTACLE_DATA, MEDIA_SUBTYPE_OBSTACLE_DATA,
                                        __exception_ptr));
        RETURN_IF_FAILED(outputPinObstacle.Create("Obstacles", pOutputType, this));
        RETURN_IF_FAILED(RegisterPin(&outputPinObstacle));

        // position input
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid **) &pDescManager, __exception_ptr));
        // get description for position pin
        tChar const* positionString = pDescManager->GetMediaDescription("tPosition");
        RETURN_IF_POINTER_NULL(positionString);

        // get mediatype for position pin
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tPosition", positionString,
                                                                 IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        // set the description for the position pin
        RETURN_IF_FAILED(
                pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&positionDescription));

        // create the input pin
        RETURN_IF_FAILED(inputPinPosition.Create("Position", pTypeSignalValue, static_cast<IPinEventSink *> (this)));
        RETURN_IF_FAILED(RegisterPin(&inputPinPosition));

        // set up camera
        try
        {
            // check if any device is availabe
            if (deviceList.get_device_count() <= 0)
            {
                CONSOLE_LOG_ERROR("No device found");
                // retur no error, or else the filter will not appear in the component list
                RETURN_NOERROR;
            }
            // use the first device
            cameraDevice = deviceList.get_device(0);
            if (nullptr == cameraDevice)
            {
                CONSOLE_LOG_ERROR("Could not get device");
                RETURN_ERROR(ERR_BAD_DEVICE);
            }

            // set effective stream frame rates
            const unsigned int minFpsStream = (modeColor.fps < modeDepth.fps) ? modeColor.fps : modeDepth.fps;
            fps = (minFpsStream < fps) ? minFpsStream : fps;
            unsigned int effectiveFpsColor = modeColor.fps;
            unsigned int effectiveFpsDepth = modeDepth.fps;
            // select minimum frame rates that are still fast enough for the required frame rate
            if (effectiveFpsColor > fps)
            {
                // any color modes above 640 width (e.g. 1920x1080) provide 15 and 30 fps
                if (modeColor.width > 640)
                {
                    if (fps <= 15) effectiveFpsColor = 15;
                    else           effectiveFpsColor = 30;
                }
                    // all others 30 or 60
                else
                {
                    if (fps <= 30) effectiveFpsColor = 30;
                    else           effectiveFpsColor = 60;
                }
            }
            if (effectiveFpsDepth > fps)
            {
                // all depth stream modes provide 30, 60 or 90 fps
                if      (fps <= 30) effectiveFpsDepth = 30;
                else if (fps <= 60) effectiveFpsDepth = 60;
                else                effectiveFpsDepth = 90;
            }

            // if the fps is different from any real stream fps, we have a synthetic fps
            // and have to take additional actions to ensure the frame rate is achieved
            isSyntheticFps = (fps != effectiveFpsDepth && fps != effectiveFpsColor);
            syntheticFrameTime = p_MICRO_IN_SEC / fps;

            // enable laser emitter, auto exposure and white balance
            cameraDevice->set_option(rs::option::r200_emitter_enabled, 1);
            cameraDevice->set_option(rs::option::r200_lr_auto_exposure_enabled, 1);
            cameraDevice->set_option(rs::option::color_backlight_compensation, 1);
            cameraDevice->set_option(rs::option::color_enable_auto_exposure, 1);
            cameraDevice->set_option(rs::option::color_enable_auto_white_balance, 1);
            // set sream parameters for real (non-synthetic) streams
            cameraDevice->enable_stream(rs::stream::color,
                                        modeColor.width, modeColor.height,
                                        rs::format::bgr8,  // bgr 8 bits per channel
                                        effectiveFpsColor);
            cameraDevice->enable_stream(rs::stream::depth,
                                        modeDepth.width, modeDepth.height,
                                        rs::format::z16,   // 16 bit depth (0 - 65535)
                                        effectiveFpsDepth);
            // start stream to make extrinsic and instrinsic parameters available
            cameraDevice->start();
            cameraDevice->wait_for_frames();
            depthIntrinsics = cameraDevice->get_stream_intrinsics(p_DEPTH_STREAM);
            colorIntrinsics = cameraDevice->get_stream_intrinsics(p_COLOR_STREAM);
            depthColorExtrinsics = cameraDevice->get_extrinsics(p_DEPTH_STREAM, p_COLOR_STREAM);
            const float depthScale = cameraDevice->get_depth_scale();
            cameraDevice->stop();
            // set minimum and maximum distance for points using the depth scale to convert
            // from 16 bit integer depth to floating depth in meters
            cameraDevice->set_option(rs::option::r200_depth_clamp_min, p_Z_MIN / depthScale);
            cameraDevice->set_option(rs::option::r200_depth_clamp_max, p_Z_MAX / depthScale);
        }
        catch (const rs::error& ex)
        {

            CONSOLE_LOG_ERROR("Error setting up device");
            CONSOLE_LOG_ERROR(ex.what());
            RETURN_ERROR(ERR_BAD_DEVICE);
        }
        // create clouds and reserve memory for points
        const std::uint32_t depthHeight = static_cast<std::uint32_t>(depthIntrinsics.height);
        const std::uint32_t depthWidth = static_cast<std::uint32_t>(depthIntrinsics.width);
        depthDataSize = depthHeight * depthWidth;
        pointCloud.reset(new pcl::PointCloud<pcl::PointXYZ>(depthWidth, depthHeight, {0.f, 0.f, 0.f}));
        pointCloudFiltered.reset(new pcl::PointCloud<pcl::PointXYZ>(depthWidth, depthHeight, {0.f, 0.f, 0.f}));
        // our clouds are always dense, e.g. will not contain any infinity or NaN values
        pointCloud->is_dense = true;
        pointCloudFiltered->is_dense = true;
        // reserve memory for color image
        rgbImage = cv::Mat(colorIntrinsics.height,
                           colorIntrinsics.width,
                           CV_8UC3,
                           cv::Scalar(0,0,0));
    }
    else if (eStage == StageGraphReady)
    {
        // prepare output image
        updateOutputImageFormat(rgbImage, outputPinRgbFormat, outputPinRgb);
        // create the thread
        tResult nResult = thread.Create(cKernelThread::TF_Suspended,
                                        static_cast<adtf::IKernelThreadFunc*>(this));
        if (IS_FAILED(nResult))
        {
            THROW_ERROR_DESC(nResult, "Failed to create threads");
        }

    }

    RETURN_NOERROR;
}

// ###############################################################################################
tResult PointClouds::OnPinEvent(IPin* pSource,
                                tInt nEventCode,
                                tInt /*nParam1*/,
                                tInt /*nParam2*/,
                                IMediaSample* pMediaSample)
{
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);

    if (IPinEventSink::PE_MediaSampleReceived == nEventCode)
    {
        if (pSource == &inputPinPosition)
        {
            float px = 0, py = 0, r = 0, s = 0, h = 0;
            {
                {
                    __adtf_sample_read_lock_mediadescription(positionDescription, pMediaSample,
                                                             pCoderInput);
                    if (!positionIds.positionIDsSet) {
                        RETURN_IF_FAILED(pCoderInput->GetID("f32x", positionIds.idF32X));
                        RETURN_IF_FAILED(pCoderInput->GetID("f32y", positionIds.idF32Y));
                        RETURN_IF_FAILED(pCoderInput->GetID("f32radius", positionIds.idF32Radius));
                        RETURN_IF_FAILED(pCoderInput->GetID("f32speed", positionIds.idF32Speed));
                        RETURN_IF_FAILED(pCoderInput->GetID("f32heading", positionIds.idF32Heading));
                        positionIds.positionIDsSet = tTrue;
                    }
                    RETURN_IF_FAILED(pCoderInput->Get(positionIds.idF32X, (tVoid *) &px));
                    RETURN_IF_FAILED(pCoderInput->Get(positionIds.idF32Y, (tVoid *) &py));
                    RETURN_IF_FAILED(pCoderInput->Get(positionIds.idF32Radius, (tVoid *) &r));
                    RETURN_IF_FAILED(pCoderInput->Get(positionIds.idF32Speed, (tVoid *) &s));
                    RETURN_IF_FAILED(pCoderInput->Get(positionIds.idF32Heading, (tVoid *) &h));
                }
            }
            bufferFront.position.point.x = px;
            bufferFront.position.point.y = py;
            bufferFront.position.radius = r;
            bufferFront.position.speed = s;
            bufferFront.position.heading = h;
            bufferSwapMutex.Enter();
            if (!firstPositionReceived) firstPositionReceived = true;
            bufferFront.swap(bufferMid);
            newBuffer = true;
            bufferSwapMutex.Leave();
        }
    }
    RETURN_NOERROR;
}

// ###############################################################################################
tResult PointClouds::ThreadFunc(cKernelThread * pThread,
                                tVoid * /*pvUserData*/,
                                tSize /*szUserData*/)
{
    const tTimeStamp frameStart = (isSyntheticFps) ? cSystem::GetTime() : 0;
    // sleep for minimum time so scheduler can reschedule thread
    cSystem::Sleep(1);
    if (pThread != &thread)
    {
        RETURN_NOERROR;
    }
    bufferSwapMutex.Enter();
    // wait for first position update
    if (!firstPositionReceived && !instantStart)
    {
        bufferSwapMutex.Leave();
        cSystem::Sleep(p_TIME_TO_WAIT_EACH_CALL_UNTIL_INITIAL_POSITION);
        RETURN_NOERROR;
    }
    // get data from main thread
    if (newBuffer)
    {
        bufferBack.swap(bufferMid);
        newBuffer = false;
    }
    bufferSwapMutex.Leave();
    // check if camera is still valid
    cameraMutex.Enter();
    if (nullptr == cameraDevice)
    {
        cameraMutex.Leave();
        CONSOLE_LOG_ERROR("Camera disappeared");
        RETURN_NOERROR;
    }
    // get frame data
    try
    {
        cameraDevice->wait_for_frames();
    }
    catch (const rs::error& err)
    {
        cameraMutex.Leave();
        CONSOLE_LOG_ERROR("Camera - wait_for_frames - error thrown: ");
        CONSOLE_LOG_ERROR(err.what());
        RETURN_NOERROR;
    }
    // todo check if wait for frames or GetTime is acting up
    //const tTimeStamp time = _clock->GetTime();
    updateRgbFrame();
    updatePointCloud();
    cameraMutex.Leave();

    // filter point cloud and generate obstacle data
    filterPointCloud();

    if (obstacleData.empty()) RETURN_NOERROR;

    // transmit obstacle and image
    RETURN_IF_FAILED(transmitObstacle(obstacleData, 0 /*time*/));
    RETURN_IF_FAILED(transmitImageRgb(rgbImage, 0 /*time*/));

#ifdef HTWK_POINTCLOUDS_DEBUG
    viewer->showCloud(pointCloudFiltered);
#endif // HTWK_POINTCLOUDS_DEBUG

    // if a fps other than the stream fps is set, wait until required frame time delta is reached
    // otherwise the wait_for_frames call ensures this
    if (isSyntheticFps)
    {
        const tTimeStamp frameDeltaRest = syntheticFrameTime - (cSystem::GetTime() - frameStart);
        if (frameDeltaRest > 0)
        {
            cSystem::Sleep(frameDeltaRest);
        }
    }

    RETURN_NOERROR;
}

// ###############################################################################################
void PointClouds::updatePointCloud()
{
    // copy points from stream to point cloud
    const rs::float3* pointData =
            static_cast<const rs::float3*>(cameraDevice->get_frame_data(p_DEPTH_STREAM));
    pointCloud->points.clear();
    for (std::size_t i = 0; depthDataSize > i; ++i)
    {
        const rs::float3& p = pointData[i];
        if (p.z > 0)
        {
            pointCloud->points.emplace_back(-p.x, -p.y, p.z);
        }
    }
    // unorganized point clouds use width as size, and a height of 1
    // organized clouds have to have a point at each grid point
    // by filtering points outside of a distance range, the organized structure is destroyed
    pointCloud->width = static_cast<std::uint32_t>(pointCloud->points.size());
    pointCloud->height = 1;
}

// ###############################################################################################
void PointClouds::updateRgbFrame()
{
    // copy color image data to cv matrix
    rgbImage.data = static_cast<uchar*>(const_cast<void*>(
            cameraDevice->get_frame_data(p_COLOR_STREAM)));
}

// ###############################################################################################
bool clusterCondition(const pcl::PointXYZ& point_a, const pcl::PointXYZ& point_b, const float squared_distance);
// ###############################################################################################
void PointClouds::filterPointCloud()
{
    defTime();

    if (pointCloud->empty()) return;

    // limit point cloud to relevant space
    timeNow();
    pcl::PassThrough<pcl::PointXYZ> ptf;
    ptf.setFilterFieldName("y");
    ptf.setFilterLimits(p_Y_MIN, p_Y_MAX);
    ptf.setInputCloud(pointCloud);
    ptf.filter(*pointCloudFiltered);
    duration("ptf: ");

    if (pointCloudFiltered->empty()) return;

    // downsample cloud using a voxel grid (3.125cm^3 minimum leaf size)
    timeNow();
    pcl::VoxelGrid<pcl::PointXYZ> vgf;
    vgf.setLeafSize(p_VOXEL_SUBSAMPLING_LEAF_SIZE_MIN,
                    p_VOXEL_SUBSAMPLING_LEAF_SIZE_MIN,
                    p_VOXEL_SUBSAMPLING_LEAF_SIZE_MIN);
    vgf.setInputCloud(pointCloudFiltered);
    vgf.filter(*pointCloudFiltered);
    duration("vgf: ");

    if (pointCloudFiltered->empty()) return;

    // get ground plane parameters
    // Hessian Normal form: n = [normal_x normal_y normal_z d]; nx = 0
    timeNow();
    const pcl::ModelCoefficients::Ptr planeParams = findGroundPlane(pointCloudFiltered);
    duration("gp: ");
    // calculate average for plane parameters to smooth jitter
    timeNow();
    planeParamAvg.add(*planeParams);
    duration("gpavg: ");

    // remove all points below ground plane
    timeNow();
    const pcl::PointIndices::Ptr pointsBelowPlane =
            belowPlane(planeParamAvg.average, pointCloudFiltered);
    duration("below: ");
    timeNow();
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(pointCloudFiltered);
    extract.setIndices(pointsBelowPlane);
    extract.setNegative(true);
    extract.filter(*pointCloudFiltered);
    duration("blwex: ");

    if (pointCloudFiltered->empty()) return;

    // remove outliers
    timeNow();
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(pointCloudFiltered);
    ror.setMinNeighborsInRadius(p_INLIER_MIN_NEIGHBORS);
    ror.setRadiusSearch(p_INLIER_SEARCH_RADIUS);
    ror.filter(*pointCloudFiltered);
    duration("ror: ");

    if (pointCloudFiltered->empty()) return;

    // conditional euclidian clustering
    timeNow();
    std::vector<pcl::PointIndices> clusters;
    pcl::ConditionalEuclideanClustering<pcl::PointXYZ> clustering;
    clustering.setConditionFunction(&clusterCondition);
    clustering.setClusterTolerance(p_CLUSTER_PRECONDITION_DISTANCE_MAX);
    clustering.setMinClusterSize(p_CLUSTER_SIZE_MIN);
    clustering.setMaxClusterSize(p_CLUSTER_SIZE_MAX);
    clustering.setInputCloud(pointCloudFiltered);
    clustering.segment(clusters);
    duration("clus: ");

    // create bounding boxes from clusters
    timeNow();
    const std::shared_ptr<std::vector<tAABB>> bboxes = getAABBs(clusters,
                                                                pointCloudFiltered,
                                                                planeParamAvg.average);
    duration("aabb: ");
    // make obstacle data from bounding boxes
    timeNow();
    makeObstacles(bboxes, bufferBack.position);
    duration("obstacles: ");
#ifdef HTWK_POINTCLOUDS_DEBUG
    timeNow();
    drawObstacles();
    duration("aabbD: ");
#endif // HTWK_POINTCLOUDS_DEBUG
}

// ###############################################################################################
const pcl::ModelCoefficients::Ptr PointClouds::findGroundPlane(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud) const
{
    static const double pp_RAD_FREEDOM = p_SAC_PLANE_ROTATION_FREEDOM * (M_PI/180.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr buffer(new pcl::PointCloud<pcl::PointXYZ>);

    // only consider a small z-slice for finding the ground plane to reduce noise and object interference
    pcl::PassThrough<pcl::PointXYZ> ptf;
    ptf.setInputCloud(inputCloud);
    ptf.setFilterFieldName("z");
    ptf.setFilterLimits(p_SAC_SPACE_Z_MIN, p_SAC_SPACE_Z_MAX);
    ptf.filter(*buffer);

    // apply RANSAC to find ground plane parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setOptimizeCoefficients(true);
    // find a plane perpendicular to a give axis
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMaxIterations(p_SAC_MAX_ITERATIONS);
    // max point distance to plane model for inclusion
    seg.setDistanceThreshold(p_SAC_PLANE_DISTANCE_MAX);
    // perpendicular to axis
    seg.setAxis(p_SAC_PLANE_PERPENDICULAR_AXIS);
    // rotation freedom (in radians) out of axis-constraint
    seg.setEpsAngle(pp_RAD_FREEDOM);
    seg.setInputCloud(buffer);

    pcl::PointIndices inliers;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.segment(inliers, *coefficients);
    return coefficients;
}

// ###############################################################################################
const pcl::PointIndices::Ptr PointClouds::belowPlane(
        const pcl::ModelCoefficients& plane,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // ensure the all required plane parameters are available (a, b, c, d and multiplier)
    if (plane.values.size() != 5)
    {
        return inliers;
    }
    const float& a = plane.values[0];
    const float& b = plane.values[1];
    const float& c = plane.values[2];
    const float& d = plane.values[3];
    const float& mul = plane.values[4];

    // maximum amount of inliers are limitied by the cloud size
    inliers->indices.reserve(cloud->size());
    const std::size_t& bsize = cloud->size();
    for (unsigned int i = 0; bsize > i; ++i)
    {
        const pcl::PointXYZ& p = cloud->points[i];
        // check if each point is below or above the ground plane,
        // and collect any point below or not far enough above the ground
        if ((a * p.x + b * p.y + c * p.z + d) * mul > -p_REMOVE_POINTS_ABOVE_GROUND_DISTANCE)
        {
            inliers->indices.emplace_back(i);
        }
    }
    return inliers;
}

// ###############################################################################################
/** checks if a point is valid for inclusion in the cluster */
bool clusterCondition(
        const pcl::PointXYZ& point_a,
        const pcl::PointXYZ& point_b,
        const float squared_distance)
{
    static const float pp_DIST_DEF = p_CLUSTER_DISTANCE_MAX * p_CLUSTER_DISTANCE_MAX;
    static const float pp_DIST_OVERLAP = p_CLUSTER_DISTANCE_MAX_FOR_XY_OVERLAP * p_CLUSTER_DISTANCE_MAX_FOR_XY_OVERLAP;
    static const float pp_DIST_Y = p_CLUSTER_DISTANCE_MAX_OVERLAP_MAX_DIST_Y;
    static const float pp_DIST_X = p_CLUSTER_DISTANCE_MAX_OVERLAP_MAX_DIST_X;

    // check if point is close enough to the cluster
    if (pp_DIST_DEF >= squared_distance)
    {
        return true;
    }
        // if it is close enough on the XY plane, reconsider it within a larger distance
        // this gives more leeway to points on the XZ plane, as the point density is lower
    else if (   pp_DIST_OVERLAP >= squared_distance
                && std::fabs(point_a.y - point_b.y) <= pp_DIST_Y
                && std::fabs(point_a.x - point_b.x) <= pp_DIST_X)
    {
        return true;
    }
    return false;
}

// ###############################################################################################
std::shared_ptr<std::vector<PointClouds::tAABB>> PointClouds::getAABBs(
        const std::vector<pcl::PointIndices>& clusters,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const pcl::ModelCoefficients& ground) const
{
    std::shared_ptr<std::vector<tAABB>> bboxes = std::make_shared<std::vector<tAABB>>();
    // ensure the all required parameters are available (a, b, c, d and multiplier)
    if (ground.values.size() != 5)
    {
        return bboxes;
    }

    const float& a = ground.values[0];
    const float& b = ground.values[1];
    const float& c = ground.values[2];
    const float& d = ground.values[3];
    const float& mul = ground.values[4];

    // amount of bounding boxes are at most the amount of clusters
    bboxes->reserve(clusters.size());
    for (const pcl::PointIndices& clus : clusters)
    {
        // create an empty bounding box
        bboxes->emplace_back();
        tAABB& bbox = bboxes->back();
        // get the first point in the cluster
        const pcl::PointXYZ& initP = cloud->points[static_cast<const std::size_t&>(clus.indices[0])];
        bbox.pMax.x = bbox.pMin.x = initP.x;
        bbox.pMax.y = bbox.pMin.y = initP.y;
        bbox.pMax.z = bbox.pMin.z = initP.z;

        // reserve memory for cluster points for convex hull later on
        if (makeHull)
        {
            bbox.points.reserve(clus.indices.size());
        }
        // find minimum and maximum points for this cluster and save as bounding box corners
        for (const int& i : clus.indices)
        {
            const pcl::PointXYZ& p = cloud->points[static_cast<const std::size_t&>(i)];
            if      (p.x > bbox.pMax.x) bbox.pMax.x = p.x;
            else if (p.x < bbox.pMin.x) bbox.pMin.x = p.x;
            if      (p.y > bbox.pMax.y) bbox.pMax.y = p.y;
            else if (p.y < bbox.pMin.y) bbox.pMin.y = p.y;
            if      (p.z > bbox.pMax.z) bbox.pMax.z = p.z;
            else if (p.z < bbox.pMin.z) bbox.pMin.z = p.z;
            if (makeHull)
            {
                bbox.points.emplace_back(p);
            }
        }
        // bounding box sizes
        bbox.sx = std::fabs(bbox.pMax.x - bbox.pMin.x);
        bbox.sy = std::fabs(bbox.pMax.y - bbox.pMin.y);
        bbox.sz = std::fabs(bbox.pMax.z - bbox.pMin.z);
        // bounding box XY area
        const float bboxA = bbox.sx * bbox.sy;
        const pcl::PointXYZ& pMin = bbox.pMin;
        // check if bounding box is withing given constraints
        if (!( bbox.sx <= p_BBOX_WIDTH_MAX
            && bbox.sy <= p_BBOX_HEIGHT_MAX
            && bbox.sx >= p_BBOX_WIDTH_MIN
            && bbox.sy >= p_BBOX_HEIGHT_MIN
            && bboxA >= p_BBOX_MIN_AREA
            // check if bounding box starts too far from the ground
            && std::fabs((a * pMin.x + b * pMin.y + c * pMin.z + d) * mul) < p_BBOX_MAX_DISTANCE_TO_GROUND))
        {
            // remove bounding box, if any constraint is violated
            bboxes->pop_back();
        }
        // pad bounding boxes
        bbox.pMin.x -= p_BBOX_PADDING;
        bbox.pMin.y -= p_BBOX_PADDING;
        bbox.pMin.z -= p_BBOX_PADDING;
        bbox.pMax.x += p_BBOX_PADDING;
        bbox.pMax.y += p_BBOX_PADDING;
        bbox.pMax.z += p_BBOX_PADDING;
    }

    return bboxes;
}

// ###############################################################################################
void PointClouds::makeObstacles(
        const std::shared_ptr<std::vector<tAABB>>& bboxes,
        const tPosition& position)
{
    obstacleData.clear();
    obstacleData.reserve(bboxes->size());

    for (const tAABB& box : *bboxes)
    {
        obstacleData.emplace_back();
        tObstacleData& obstacle = obstacleData.back();
        obstacle.id = obstacleId;
        obstacle.height = box.sy;
        obstacle.radius = ((box.sx > box.sz) ? box.sx : box.sz) / 2.f;
        // bounding box XZ center point
        const float mx = (-box.pMin.x + -box.pMax.x) / 2.f;
        const float mz = (box.pMin.z + box.pMax.z) / 2.f;
        obstacle.distance = mz;
        // map center point to world coordinates
        const cv::Point2f mappedPoint = htwk::translateAndRotate2DPoint(
                cv::Point2f(mx, mz),
                static_cast<float>(position.heading - M_PI_2),
                p_RS_TO_IMU,
                position.point);
        obstacle.obstacle.x = mappedPoint.x;
        obstacle.obstacle.y = mappedPoint.y;

        // map bounding box to color image space
        const rs::float3 minC = depthColorExtrinsics.transform({-box.pMin.x, -box.pMin.y, box.pMin.z});
        const rs::float3 maxC = depthColorExtrinsics.transform({-box.pMax.x, -box.pMax.y, box.pMax.z});
        const rs::float2 min = colorIntrinsics.project({minC.x, minC.y, minC.z});
        const rs::float2 max = colorIntrinsics.project({maxC.x, maxC.y, maxC.z});
        obstacle.rgbROI.topLeft.x = max.x;
        obstacle.rgbROI.topLeft.y = max.y;
        obstacle.rgbROI.bottomRight.x = min.x;
        obstacle.rgbROI.bottomRight.y = min.y;

        // get convex hull in image space
        if (makeHull)
        {
            std::vector<cv::Point> points;
            points.reserve(box.points.size());
            std::vector<cv::Point> hull;
            for (const pcl::PointXYZ& p : box.points)
            {
                // map point to color image space
                const rs::float3 pC = depthColorExtrinsics.transform({-p.x, -p.y, p.z});
                const rs::float2 pp = colorIntrinsics.project({pC.x, pC.y, pC.z});
                const int px = static_cast<int>(pp.x - max.x + 0.5f);
                const int py = static_cast<int>(pp.y - max.y + 0.5f);
                points.emplace_back(px, py);
            }
            cv::convexHull(points, hull);
            std::size_t i = 0;
            for (const cv::Point& p : hull)
            {
                obstacle.hull[i] = p;
                if (i >= OBSTACLE_HULL_SIZE_MAX) break;
                i++;
            }
            obstacle.hullSize = i;
        }

        obstacleId++;
    }
}

// ###############################################################################################
void PointClouds::drawObstacles()
{
    std::vector<cv::Point> hullBuffer;
    for (const tObstacleData& obst : obstacleData)
    {
        const int x = static_cast<int>(obst.rgbROI.topLeft.x + 0.5f);
        const int y = static_cast<int>(obst.rgbROI.topLeft.y + 0.5f);
        const int width = static_cast<int>(obst.rgbROI.bottomRight.x - obst.rgbROI.topLeft.x + 0.5f);
        const int height = static_cast<int>(obst.rgbROI.bottomRight.y - obst.rgbROI.topLeft.y + 0.5f);
        // draw bbox
        cv::rectangle(
                rgbImage,
                cv::Rect(x, y, width, height),
                cv::Scalar(
                        0,
                        static_cast<int>(255 * (obst.distance / p_Z_MAX)),
                        static_cast<int>(255 - 255 * (obst.distance / p_Z_MAX))),
                2);
        // draw hull
        if (makeHull)
        {
            hullBuffer.clear();
            hullBuffer.reserve(obst.hullSize);
            for (std::size_t i = 0; obst.hullSize > i; ++i)
            {
                hullBuffer.emplace_back(
                        obst.hull[i].x + obst.rgbROI.topLeft.x,
                        obst.hull[i].y + obst.rgbROI.topLeft.y);
            }
            const cv::Point* polyP = hullBuffer.data();
            const cv::Point* const* polyPP = &polyP;
            const int npt[] = {static_cast<int>(obst.hullSize)};
            cv::polylines(rgbImage, polyPP, npt, 1, true, cv::Scalar(255,0,255), 1);
        }
    }
}

// ###############################################################################################
tResult PointClouds::transmitObstacle(
        const std::vector<tObstacleData>& obstacle,
        const tTimeStamp& time)
{
    if (outputPinObstacle.IsConnected())
    {
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        RETURN_IF_FAILED(pMediaSample->Update(time, obstacle.data(),
                                              tInt(obstacle.size() * sizeof(tObstacleData)),
                                              IMediaSample::tFlags::MSF_None));
        RETURN_IF_FAILED(outputPinObstacle.Transmit(pMediaSample));
    }
    RETURN_NOERROR;
}

// ###############################################################################################
tResult PointClouds::transmitImageRgb(
        const cv::Mat& image,
        const tTimeStamp& time)
{
    if (outputPinRgb.IsConnected())
    {
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**) &pMediaSample));
        RETURN_IF_FAILED(pMediaSample->AllocBuffer(outputPinRgbFormat.nSize));
        RETURN_IF_FAILED(pMediaSample->Update(time, image.data,
                                              outputPinRgbFormat.nSize,
                                              IMediaSample::MSF_None));
        RETURN_IF_FAILED(outputPinRgb.Transmit(pMediaSample));
    }
    RETURN_NOERROR;
}

// ###############################################################################################

