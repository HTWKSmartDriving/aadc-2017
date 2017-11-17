#ifndef HTWK_POINTCLOUDS_CLASS_H
#define HTWK_POINTCLOUDS_CLASS_H

// ###############################################################################################
#define HTWK_POINTCLOUDS_DEBUG
#undef HTWK_POINTCLOUDS_DEBUG

// ###############################################################################################
#include "stdafx.h"
#include "../../HTWK_Types/ObstacleData.h"
#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#ifdef HTWK_POINTCLOUDS_DEBUG
    #include <pcl/visualization/cloud_viewer.h>
#endif // HTWK_POINTCLOUDS_DEBUG

// ###############################################################################################
#define OID_ADTF_FILTER_DEF "htwk.PointClouds"
#define ADTF_FILTER_DESC "HTWK Point Clouds"
#define ADTF_FILTER_VERSION_SUB_NAME "PointClouds"
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"
#define ADTF_FILTER_VERSION_STRING "1.0.0"
#define ADTF_FILTER_VERSION_Major 1
#define ADTF_FILTER_VERSION_Minor 0
#define ADTF_FILTER_VERSION_Build 0
#define ADTF_FILTER_VERSION_LABEL ""

// ###############################################################################################
// necessary because WinGDI.h redefines it (then don't use the same name, sheesh)
#undef GetObject

// ###############################################################################################
/**
 * Realsense camera device with object detection based on point clouds.
 */
class PointClouds : public adtf::cFilter, public adtf::IKernelThreadFunc
{
public:
ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF,
                    ADTF_FILTER_DESC,
                    adtf::OBJCAT_CameraDevice,
                    ADTF_FILTER_VERSION_SUB_NAME,
                    ADTF_FILTER_VERSION_Major,
                    ADTF_FILTER_VERSION_Minor,
                    ADTF_FILTER_VERSION_Build,
                    ADTF_FILTER_VERSION_LABEL);

public:
    /** video stream mode parameters */
    typedef struct tVideoMode
    {
        /** stream width */
        unsigned int width;
        /** stream height */
        unsigned int height;
        /** stream frame rate */
        unsigned int fps;
        tVideoMode(const unsigned int& w, const unsigned int& h, const unsigned int& r) :
                width(w), height(h), fps(r) {}
    } tVideoMode;

    PointClouds(const tChar* __info);
    virtual ~PointClouds() override;

    virtual tResult PropertyChanged(const tChar* strName) override;
    virtual tResult OnPinEvent(
            IPin* pSource,
            tInt nEventCode,
            tInt nParam1,
            tInt nParam2,
            IMediaSample* pMediaSample) override;
    virtual tResult ThreadFunc(
            cKernelThread* pThread,
            tVoid* pvUserData,
            tSize szUserData) override;

protected:
    virtual tResult Init(tInitStage eStage, __exception = NULL) override;
    virtual tResult Start(__exception = NULL) override;
    virtual tResult Stop(__exception = NULL) override;
    virtual tResult Shutdown(tInitStage eStage, __exception = NULL) override;


private:
    /** 3D axis aligned bounding box */
    typedef struct tAABB
    {
        /** bottom right corner */
        pcl::PointXYZ pMax;
        /** top left corner */
        pcl::PointXYZ pMin;
        /** width */
        float sx;
        /** height */
        float sy;
        /** depth */
        float sz;
        /** points within bounding box */
        std::vector<pcl::PointXYZ> points;
    } tAABB;

    /** world position data */
    typedef struct tPosition {
        /** position */
        cv::Point2f point;
        /** position probability radius */
        float radius = 0;
        /** movement speed */
        float speed = 0;
        /** rotation of front vector in radians */
        float heading = 0;
    } tPosition;

    /** position media type IDs */
    typedef struct tPositionIds
    {
        /** the id for the f32x of the media description for output pin */
        tBufferID idF32X;
        /** the id for the f32y of the media description for output pin */
        tBufferID idF32Y;
        /** the id for the af32radius of the media description for output pin */
        tBufferID idF32Radius;
        /** the id for the af32speed of the media description for output pin */
        tBufferID idF32Speed;
        /** the id for the af32heading of the media description for output pin */
        tBufferID idF32Heading;
        /** set to true once IDs have been set */
        tBool positionIDsSet = false;
    } tPositionIds;

    /** buffer for exchanging data between processing and main thread */
    typedef struct tBuffer
    {
        /** position data */
        tPosition position;
        /** swap two buffers */
        void swap(tBuffer& other)
        {
            const tPosition pos = other.position;
            other.position = position;
            position = pos;
        }
    } tBuffer;

    /** transmit rgb image on image output pin
     * @param image the image to transmit
     * @param time  the timestamp to transmit
     * @return result code
     * */
    tResult transmitImageRgb(const cv::Mat& image, const tTimeStamp& time);
    /** transmit obstacle data on obstacle output pin
     * @param obstacle  a vector of obstacles to transmit
     * @param time      the timestamp to transmit
     * @param result code
     * */
    tResult transmitObstacle(const std::vector<tObstacleData>& obstacle, const tTimeStamp& time);
    /** generate point cloud from depth stream */
    void updatePointCloud();
    /** copy color frame data */
    void updateRgbFrame();
    /** process cloud and generate obstacles */
    void filterPointCloud();
    /** find the ground plane
     *  @param inputCloud cloud to find a ground plane in
     **/
    const pcl::ModelCoefficients::Ptr findGroundPlane(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud) const;
    /** find all points below a plane */
    const pcl::PointIndices::Ptr belowPlane(
            const pcl::ModelCoefficients& plane,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;
    /** get bounding boxes of clusters in given cloud
     * @param clusters  clusters to make bboxes of, adhering to given constraints
     * @param cloud     cloud containing the clusters
     * @param ground    ground plane parameters
     * @return bounding boxes
     */
    std::shared_ptr<std::vector<tAABB>> getAABBs(
            const std::vector<pcl::PointIndices>& clusters,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
            const pcl::ModelCoefficients& ground) const;
    /** create obstacle data from bounding boxes
     * @param boxes     bounding boxes
     * @param position  current world position
     */
    void makeObstacles(
            const std::shared_ptr<std::vector<tAABB>>& boxes,
            const tPosition& position);
    /** draw obstacle bounding boxes on color image */
    void drawObstacles();

    /** input pin for position data */
    cInputPin inputPinPosition;
    /** position media description */
    cObjectPtr<IMediaTypeDescription> positionDescription;
    /** position media description IDs */
    tPositionIds positionIds;
    /** image output */
    cVideoPin outputPinRgb;
    /** format of output image*/
    tBitmapFormat outputPinRgbFormat;
    /** output pin for obstacle data */
    cOutputPin outputPinObstacle;

    /** color stream mode */
    tVideoMode modeColor;
    /** depth stream mode */
    tVideoMode modeDepth;
    /** target effective processing fps */
    unsigned int fps;
    /** if target fps does not correspond to real stream fps */
    bool isSyntheticFps;
    /** target frame time corresponding to synthetic fps */
    tTimeStamp syntheticFrameTime;

    /** if filter should start detection without waiting for first position data */
    bool instantStart;
    /** true once first position data was received */
    bool firstPositionReceived;

    /** detected objects as obstacle data vector */
    std::vector<tObstacleData> obstacleData;
    /** id for next detected obstacle */
    tUInt64 obstacleId;
    /** make convex hulls for obstacles */
    bool makeHull;

    /** realsense device context, makes devices available */
    rs::context deviceList;
    /** realsense camera device */
    rs::device* cameraDevice;
    /** depth stream intrinsics */
    rs::intrinsics depthIntrinsics;
    /** color stream intrinsics */
    rs::intrinsics colorIntrinsics;
    /** depth to color stream extrinsics */
    rs::extrinsics depthColorExtrinsics;
    /** amount of depth data */
    std::uint32_t depthDataSize;
    /** color image buffer */
    cv::Mat rgbImage;
    /** input point cloud */
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;
    /** output point cloud */
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFiltered;

    /** critical section for camera device access */
    cCriticalSection cameraMutex;

    /** front buffer used by main threaad */
    tBuffer bufferFront;
    /** mid buffer used for exchanged between front and back */
    tBuffer bufferBack;
    /** back buffer used by processing thread */
    tBuffer bufferMid;
    /** set to true when front places new data in mid, false when back gets data from mid */
    tBool newBuffer;

    /** critical section for buffer swapping */
    cCriticalSection bufferSwapMutex;
    /** server communication and preprocessing threawd */
    cKernelThread thread;

    /** computes parameter averages for plane models */
    typedef struct tParameterAverage
    {
        /** kernel size for averaging */
        static const std::size_t SIZE = 6;
        /** amount of independent input plane parameters */
        static const std::size_t DATA_SIZE = 4;
        /** plane parameters */
        std::vector<pcl::ModelCoefficients> data;
        /** average of plane parameters including a precomputed multiplier for fast usage */
        pcl::ModelCoefficients average;
        /** position in kernel ring buffer */
        std::size_t pos;
        /** initialize memory */
        tParameterAverage()
        {
            // generate empty ring buffer
            for (std::size_t i = 0; SIZE > i; ++i)
            {
                pcl::ModelCoefficients d;
                for (std::size_t j = 0; DATA_SIZE > j; ++j)
                {
                    d.values.emplace_back(0);
                }
                data.emplace_back(d);
            }
            // generate empty average
            for (std::size_t j = 0; DATA_SIZE + 1 > j; ++j)
            {
                average.values.emplace_back(0);
            }
            pos = 0;
        }

        /** add a set of plane parameters
         *  @param coeff parameters
         */
        inline void add(const pcl::ModelCoefficients& coeff)
        {
            // check if all parameters are available
            if (coeff.values.size() < 4)
            {
                return;
            }
            // save parameters
            data[pos] = coeff;
            // increase ring buffer pointer
            pos++;
            pos = pos % data.size();
            // prepare accumulator
            pcl::ModelCoefficients acc;
            acc.values = {0,0,0,0};
            // accumulate parameters
            for (pcl::ModelCoefficients& d : data)
            {
                for (std::size_t j = 0; DATA_SIZE > j; ++j)
                {
                    acc.values[j] += d.values[j];
                }
            }
            // average accumulated parameters
            for (std::size_t j = 0; DATA_SIZE > j; ++j)
            {
                average.values[j] = acc.values[j] / SIZE;
            }
            // precompute multiplier
            const float& a = average.values[0];
            const float& b = average.values[1];
            const float& c = average.values[2];
            const float mul = 1.f / std::sqrt(a * a + b * b + c * c);
            average.values[DATA_SIZE] = mul;
        }

    } tParameterAverage;
    /** used to average ground plane over multiple frames */
    tParameterAverage planeParamAvg;

#ifdef HTWK_POINTCLOUDS_DEBUG
    std::shared_ptr<pcl::visualization::CloudViewer> viewer;
#endif // HTWK_POINTCLOUDS_DEBUG
};


// ###############################################################################################
#endif // HTWK_POINTCLOUDS_CLASS_H
