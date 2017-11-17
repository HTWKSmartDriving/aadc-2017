#ifndef HTWK_CLASSIFICATIONSERVER_CLASS_H
#define HTWK_CLASSIFICATIONSERVER_CLASS_H

// ###############################################################################################
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TSimpleServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TBufferTransports.h>
#include <thrift/transport/TSocket.h>

#include "stdafx.h"
#include <opencv2/opencv.hpp>

#include "ThriftInterface_Service.h"
#include "ThriftInterface_types.h"

#include "../../HTWK_Types/ObstacleData.h"

// ###############################################################################################
#define OID_ADTF_FILTER_DEF "htwk.ClassificationServer"
#define ADTF_FILTER_DESC "HTWK External Classification Server"
#define ADTF_FILTER_VERSION_SUB_NAME "ClassificationServer"
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"
#define ADTF_FILTER_VERSION_STRING "1.0.0"
#define ADTF_FILTER_VERSION_Major 1
#define ADTF_FILTER_VERSION_Minor 0
#define ADTF_FILTER_VERSION_Build 0
#define ADTF_FILTER_VERSION_LABEL "this filter creates the THRIFT RPC Client which can receive data and transmit image data."

// ###############################################################################################
// necessary because WinGDI.h redefines it (then don't use the same name, sheesh)
#undef GetObject

// ###############################################################################################
class ClassificationServer : public adtf::cFilter, public adtf::IKernelThreadFunc
{
public:
    ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF,
                        ADTF_FILTER_DESC,
                        adtf::OBJCAT_Auxiliary,
                        ADTF_FILTER_VERSION_SUB_NAME,
                        ADTF_FILTER_VERSION_Major,
                        ADTF_FILTER_VERSION_Minor,
                        ADTF_FILTER_VERSION_Build,
                        ADTF_FILTER_VERSION_LABEL
                       );

    /** Thrift port property name */
    static const cString propThriftPortName;
    /** Thrift port property description */
    static const cString propThriftPortDesc;
    /** Thrift port property default value */
    static const tInt propThriftPortDefault;

    /** Thrift address property name */
    static const cString propThriftAddressName;
    /** Thrift address property description */
    static const cString propThriftAddressDesc;
    /** Thrift address property default value */
    static const cString propThriftAddressDefault;

    /** Keras model property name */
    static const cString propModelName;
    /** Keras model property description */
    static const cString propModelDesc;
    /** Keras model property default value */
    static const cString propModelDefault;

    /** Convex hull background removal property name */
    static const cString propHullName;
    /** Convex hull background removal property description */
    static const cString propHullDesc;
    /** Convex hull background removal property default value */
    static const tBool propHullDefault;

    /** Image normalization property name */
    static const cString propNormName;
    /** Image normalization property description */
    static const cString propNormDesc;
    /** Image normalization property default value */
    static const tBool propNormDefault;

public:
    ClassificationServer(const tChar* __info);
    virtual ~ClassificationServer() override;

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
    tBool CheckAndConnect();
    /** Update image format for incoming RGB images */
    tResult UpdateInputImageFormat();
    /** Swap image, roi and obstacle data in from front to back buffer */
    tResult Process();
    const std::shared_ptr<thrift_interface::TDataResultList> RunClassification(
            const cv::Mat& image);
    /** Transmits obstacle data with complemented classifications */
    tResult TransmitObstacles(
            const std::vector<tObstacleData>& obstacles,
            const tTimeStamp& time);
    tResult TransmitDebugImage(const cv::Mat& image, const tTimeStamp& time);

    /** Map classification label id to obstacle type enum */
    ObstacleType mapLabelToType(const tInt16 label) const;
    /** Map obstacle type enum to classification label id */
    tInt16 mapTypeToLabel(const ObstacleType type) const;
    /** Map label name to obstacle type enum */
    ObstacleType mapStringToType(const std::string& str) const;
    /** Map obstacle type enum to label name */
    std::string mapTypeToString(const ObstacleType type) const;
    /** Map label name to label id */
    tInt16 mapStringToLabel(const std::string& str) const;
    /** Map label id to label name */
    std::string mapLabelToString(const tInt16 label) const;

    /** holds all filter properties */
    typedef struct tFilterProperties
    {
        /** thrift server port number */
        tInt port = 0;
        /** thrift server address */
        cString address = "";
        /** path to a keras model file */
        cFilename model = "";
        /** Set to true when the convex hull should be used to remove the background */
        tBool useHull = false;
        /** Set to true when the cropped image should be normalized before classification */
        tBool doNormalize = false;
    } tFilterProperties;
    /** holds all filter instance properties */
    tFilterProperties filterProperties;

    /** Set to true when the model path property changes, so it can be loaded by the server */
    bool reloadModel;

    /** holds one data set required for classificaton */
    typedef struct tClassificationBuffer
    {
        /** an image containing ROIs to be cropped and classified */
        cv::Mat image = cv::Mat();
        /** the timestamp of the data capture */
        tTimeStamp time;
        /** obstacle data to be supplemented with classification data */
        std::vector<tObstacleData> obstacle;
        /** swap to buffer objects */
        void swap(tClassificationBuffer& other)
        {
            cv::swap(image, other.image);
            obstacle.swap(other.obstacle);
            tTimeStamp tmpTime = time;
            time = other.time;
            other.time = tmpTime;
        }
    } tClassificationBuffer;

    /** front buffer, used by main thread */
    tClassificationBuffer bufferFront;
    /** back buffer, used by ThreadFunc for processing and transmission */
    tClassificationBuffer bufferBack;
    tClassificationBuffer bufferMid;
    /** set to true if data in the front buffer is updated */
    bool newBuffer;

    /** input pin for image data */
    cVideoPin inputPinImage;
    /** image format of the input image */
    tBitmapFormat inputPinImageFormat;
    bool obstaclesUpdated = false;
    bool imageUpdated = false;
    /** input pin for obstacle data, will be supplemented with classification data on output */
    cInputPin inputPinObstacles;

    /** output pin for obstacle data, including classification */
    cOutputPin outputPinObstacles;

    /** output pin for debug image */
    cVideoPin outputPinDebugImage;
    /** image format of debug output image */
    tBitmapFormat outputPinDebugImageFormat;
    bool formatHasBeenSent;
    cv::Mat debugImage;

    typedef struct tThriftConnection
    {
        /** the external interface thrift client */
        std::shared_ptr<thrift_interface::ThriftInterface_ServiceClient> client;
        /** thrift message buffer for image data */
        thrift_interface::TDataRaw data;
        /** thrift message buffer to image parameters */
        thrift_interface::TImageParams parameters;
    } tThriftConnection;

    /** holds thrift client and message buffers */
    tThriftConnection connection;

    /** critical section for buffer swapping */
    cCriticalSection bufferSwapMutex;

    /** server communication and preprocessing threawd */
    cKernelThread thread;

};

// ###############################################################################################
#endif // HTWK_CLASSIFICATIONSERVER_CLASS_H
