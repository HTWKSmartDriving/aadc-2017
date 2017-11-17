#include "ClassificationServer.h"
#include "../../HTWK_Utils/ImageHelper.h"
#include "../../HTWK_Utils/VectorHelper.h"
// ###############################################################################################

#define CONSOLE_LOG(_text, _log_level) if (m_filterProperties.enableDebugOutput) { LOG_FN_OUTPUT((_text), _log_level); }
#define CONSOLE_LOG_INFO(_text)      CONSOLE_LOG(_text, adtf_util::LOG_LVL_INFO)
#define CONSOLE_LOG_WARNING(_text)   CONSOLE_LOG(_text, adtf_util::LOG_LVL_WARNING)
#define CONSOLE_LOG_DUMP(_text)      CONSOLE_LOG(_text, adtf_util::LOG_LVL_DUMP)
#define CONSOLE_LOG_ERROR(_text)     CONSOLE_LOG(_text, adtf_util::LOG_LVL_ERROR)

ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC,
                   OID_ADTF_FILTER_DEF,
                   ClassificationServer)

// ###############################################################################################
const cString ClassificationServer::propThriftPortName = "Port";
const cString ClassificationServer::propThriftPortDesc = "Thrift port number";
const tInt ClassificationServer::propThriftPortDefault = 1833;

const cString ClassificationServer::propThriftAddressName = "Address";
const cString ClassificationServer::propThriftAddressDesc = "Thrift address";
const cString ClassificationServer::propThriftAddressDefault = "localhost";

const cString ClassificationServer::propModelName = "Model";
const cString ClassificationServer::propModelDesc = "Path to a keras model";
const cString ClassificationServer::propModelDefault = "/home/aadc/model/model.h5";

const cString ClassificationServer::propHullName = "Use convex hull";
const cString ClassificationServer::propHullDesc = "Use convex hull to remove background before classification";
const tBool ClassificationServer::propHullDefault = tFalse;

const cString ClassificationServer::propNormName = "Normalize image";
const cString ClassificationServer::propNormDesc = "Normalze bbox cropped images (usefull for direct sunlight)";
const tBool ClassificationServer::propNormDefault = tFalse;

// ###############################################################################################
ClassificationServer::ClassificationServer(const tChar *__info) :
        cFilter(__info),
        reloadModel(true),
        newBuffer(false),
        formatHasBeenSent(false) {
    SetPropertyInt(propThriftPortName, propThriftPortDefault);
    SetPropertyStr(propThriftPortName + NSSUBPROP_DESCRIPTION, propThriftPortDesc);

    SetPropertyStr(propThriftAddressName, propThriftAddressDefault);
    SetPropertyStr(propThriftAddressName + NSSUBPROP_DESCRIPTION, propThriftAddressDesc);

    SetPropertyStr(propModelName, propModelDefault);
    SetPropertyBool(propModelName + NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr(propModelName + NSSUBPROP_DESCRIPTION, propModelDesc);

    SetPropertyBool(propHullName, propHullDefault);
    SetPropertyStr(propHullName + NSSUBPROP_DESCRIPTION, propHullDesc);

    SetPropertyBool(propNormName, propNormDefault);
    SetPropertyStr(propNormName + NSSUBPROP_DESCRIPTION, propNormDesc);
}

// ###############################################################################################
ClassificationServer::~ClassificationServer() {}

// ###############################################################################################
tResult ClassificationServer::Start(__exception) {
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));
    formatHasBeenSent = false;
    imageUpdated = false;
    obstaclesUpdated = false;
    PropertyChanged(propNormName);
    PropertyChanged(propHullName);
    PropertyChanged(propModelName);
    PropertyChanged(propThriftAddressName);
    PropertyChanged(propThriftPortName);
    // start the thread if not running by now
    if (thread.GetState() == cKernelThread::TS_Suspended) {
        thread.Run(tTrue);
    }
    RETURN_NOERROR;
}

// ###############################################################################################
tResult ClassificationServer::Stop(__exception) {
    //suspend the thread
    if (thread.GetState() == cKernelThread::TS_Running) {
        thread.Suspend(tTrue);
    }
    return cFilter::Stop(__exception_ptr);
}

// ###############################################################################################
tResult ClassificationServer::Shutdown(tInitStage eStage, __exception) {
    // release the thread
    if (eStage == StageNormal) {
        thread.Terminate(tTrue);
        thread.Release();
    }
    return cFilter::Shutdown(eStage, __exception_ptr);
}

// ###############################################################################################
tResult ClassificationServer::OnPinEvent(
        IPin *pSource,
        tInt nEventCode,
        tInt /*nParam1*/,
        tInt /*nParam2*/,
        IMediaSample *pMediaSample) {
    RETURN_IF_POINTER_NULL(pSource);
    if (IPinEventSink::PE_MediaSampleReceived == nEventCode) {
        RETURN_IF_POINTER_NULL(pMediaSample);
        // image data
        if (!imageUpdated && pSource == &inputPinImage) {
            // check if we have a valid image format by now
            if (inputPinImageFormat.nWidth == 0 || inputPinImageFormat.nHeight == 0) {
                RETURN_IF_FAILED(UpdateInputImageFormat());
            }
            // copy buffer to cv matrix and keep image timestamp
            if (inputPinImageFormat.nSize != 0) {
                RETURN_IF_FAILED(bufferToMat(inputPinImage.GetFormat(), pMediaSample,
                                             bufferFront.image, inputPinImageFormat));
            }
            imageUpdated = true;
        }
            // obstacle data
        else if (!obstaclesUpdated && pSource == &inputPinObstacles) {
            htwk::recieveVector(pMediaSample, bufferFront.obstacle);
            bufferFront.time = pMediaSample->GetTime();
            obstaclesUpdated = true;
        }

        // process once data updated
        if (obstaclesUpdated && imageUpdated) {
            Process();
            obstaclesUpdated = false;
            imageUpdated = false;
        }
    } else if (nEventCode == IPinEventSink::PE_MediaTypeChanged) {
        if (pSource == &inputPinImage) {
            RETURN_IF_FAILED(UpdateInputImageFormat());
        }
    }
    RETURN_NOERROR;
}

// ###############################################################################################
tResult ClassificationServer::Process() {
    // stuffed bytes cannot be processed correctly
    if (inputPinImageFormat.nWidth * inputPinImageFormat.nBitsPerPixel / 8 != inputPinImageFormat.nBytesPerLine) {
        RETURN_ERROR(ERR_NOT_SUPPORTED);
    }

    //enter mutex and swap front/back buffers
    bufferSwapMutex.Enter();
    bufferFront.swap(bufferMid);
    newBuffer = true;
    bufferSwapMutex.Leave();

    RETURN_NOERROR;
}

// ###############################################################################################
bool scoreComparator(const tClassification& i, const tClassification& j) {
    return (i.score > j.score);
}

// ###############################################################################################
tResult ClassificationServer::ThreadFunc(
        cKernelThread *pThread,
        tVoid * /*pvUserData*/,
        tSize /*szUserData*/) {

    cSystem::Sleep(1);
    if (pThread != &thread)
    {
        RETURN_NOERROR;
    }

    if (!CheckAndConnect()) {
        //no client open
        cSystem::Sleep(1000000);
        RETURN_NOERROR;
    }

    // lock buffer swap mutex
    bufferSwapMutex.Enter();
    // check if new data has been written to the buffer
    if (!newBuffer) {
        //nothing to transmit
        bufferSwapMutex.Leave();
        cSystem::Sleep(15000); // 15ms -> 30fps (adhering to nyquist sampling theorem)
        RETURN_NOERROR;
    }
    bufferBack.swap(bufferMid);
    newBuffer = false;
    bufferSwapMutex.Leave();

    bool debugIsConnected = false;
    if (outputPinDebugImage.IsConnected() && !bufferBack.image.empty()) {
        debugIsConnected = true;
        debugImage = bufferBack.image.clone();
    }
    try {
        // iterate over ROIs
        for (auto &it : bufferBack.obstacle) {
            // check for valid ROI coords
            const cv::Rect roi(it.rgbROI.topLeft, it.rgbROI.bottomRight);
            if (!( 0 <= roi.x && 0 < roi.width  && roi.x + roi.width  <= bufferBack.image.cols
                && 0 <= roi.y && 0 < roi.height && roi.y + roi.height <= bufferBack.image.rows))
                continue;
            // crop ROI
            cv::Mat cropped;
            if (filterProperties.useHull && it.hullSize > 2) {
                cv::Mat mask(roi.height, roi.width, CV_8UC1, cv::Scalar(0));
                cv::fillConvexPoly(mask, &it.hull[0], static_cast<int>(it.hullSize), cv::Scalar(255));
                cropped = cv::Mat(roi.height, roi.width, CV_8UC3, cv::Scalar(104, 117, 124));
                bufferBack.image(roi).copyTo(cropped, mask);
            } else {
                cropped = bufferBack.image(roi).clone();
            }
            if (filterProperties.doNormalize)
            {
                cv::normalize(cropped, cropped, 255, 1, cv::NORM_MINMAX);
            }
            // classify cropped image
            ObstacleType topClassType = ObstacleType::NONE;
            tFloat topClassScore = 0.;
            const std::shared_ptr<thrift_interface::TDataResultList> response =
                    RunClassification(cropped);
            if (response->size() >= 5) {
                // complement obstacle data
                if (outputPinDebugImage.IsConnected() || outputPinObstacles.IsConnected()) {
                    std::vector<tClassification> vectorToSort;
                    for (size_t i = 0; i < response->size(); ++i) {
                        const thrift_interface::TDataResult &res = response->at(i);
                        tClassification tmpClassification;
                        tmpClassification.type = mapLabelToType(res.classification);
                        tmpClassification.score = res.probability;
                        vectorToSort.emplace_back(tmpClassification);
                    }
                    std::sort(vectorToSort.begin(), vectorToSort.end(), scoreComparator);

                    for (size_t i = 0; i < 5; ++i) {
                        it.classification[i] = vectorToSort[i];
                    }
                    topClassType = vectorToSort[0].type;
                    topClassScore = vectorToSort[0].score;
                }
                // draw bboxes and scores in debug image
                if (debugIsConnected) {
                    const std::string className = mapTypeToString(topClassType);
                    cv::putText(debugImage, className + "::" + std::to_string(topClassScore),
                                cv::Point(roi.x - 2, roi.y - 10), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(255, 0, 255), 3);
                    cv::rectangle(debugImage, roi, cv::Scalar(255, 0, 255), 3);
                }
            }
        }
        // transmit the obstacle vector
        TransmitObstacles(bufferBack.obstacle, bufferBack.time);
        // transmit debug image
        TransmitDebugImage(debugImage, bufferBack.time);
    }
    catch (const apache::thrift::TException &except) {
        //send method quit with exception so write it here to console
        LOG_ERROR("Thrift client could not send to server");
        LOG_ERROR(cString::Format("EXCEPTION: %s", except.what()));
        cSystem::Sleep(1000000);
    }
    RETURN_NOERROR;
}

// ###############################################################################################
const std::shared_ptr<thrift_interface::TDataResultList> ClassificationServer::RunClassification(
        const cv::Mat &image) {
    // fill message buffer with image data and image parameters
    std::vector<unsigned char> outData;
    // encode image data
    static const std::vector<int> p_ENCODE_PARAMS = {};
    if (cv::imencode("*.ppm", image, outData, p_ENCODE_PARAMS)) {
        // set image parameters
        connection.parameters.__set_bytesPerPixel(inputPinImageFormat.nBitsPerPixel);
        connection.parameters.__set_height(image.rows);
        connection.parameters.__set_width(image.cols);
        // set image data
        connection.data.raw_data.clear();
        connection.data.raw_data.assign((const char *) (outData.data()), outData.size());
    }
    std::shared_ptr<thrift_interface::TDataResultList> response =
            std::make_shared<thrift_interface::TDataResultList>();
    // send data to python and wait for response
    connection.client->rawData(*response, thrift_interface::TransportDef::IMAGEDATA,
                               connection.data, connection.parameters);
    return response;
}

// ###############################################################################################
tResult ClassificationServer::TransmitObstacles(
        const std::vector<tObstacleData> &obstacles,
        const tTimeStamp &time) {
    if (outputPinObstacles.IsConnected()) {
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));
        RETURN_IF_FAILED(pMediaSample->Update(time, obstacles.data(),
                                              tInt(obstacles.size() * sizeof(tObstacleData)),
                                              0));
        RETURN_IF_FAILED(outputPinObstacles.Transmit(pMediaSample));
    }
    RETURN_NOERROR;
}

// ###############################################################################################
tResult ClassificationServer::TransmitDebugImage(const cv::Mat& image, const tTimeStamp& time)
{
    if (outputPinDebugImage.IsConnected()) {
        if (!formatHasBeenSent) {
            updateOutputImageFormat(image, outputPinDebugImageFormat, outputPinDebugImage);
            formatHasBeenSent = true;
        }
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));
        RETURN_IF_FAILED(pMediaSample->AllocBuffer(outputPinDebugImageFormat.nSize));
        RETURN_IF_FAILED(pMediaSample->Update(time, image.data,
                                              outputPinDebugImageFormat.nSize,
                                              IMediaSample::MSF_None));
        RETURN_IF_FAILED(outputPinDebugImage.Transmit(pMediaSample));
    }
    RETURN_NOERROR;
}

// ###############################################################################################
tBool ClassificationServer::CheckAndConnect() {
    tBool clientIsOpen = tFalse;
    try {
        //try to open if not opened
        if (!connection.client->getOutputProtocol()->getOutputTransport()->isOpen()) {
            //open the port for the client
            connection.client->getOutputProtocol()->getOutputTransport()->open();
            // todo get label map from python server: thriftClient->getLabelMap(TLabelMap& map)
            if (connection.client->getOutputProtocol()->getOutputTransport()->isOpen()) {
                connection.client->loadModel(std::string(filterProperties.model));
                reloadModel = false;
            }
        }
        //verify
        if (connection.client->getOutputProtocol()->getOutputTransport()->isOpen()) {
            clientIsOpen = tTrue;
        }
    }
    catch (const apache::thrift::TException& except) {
        LOG_ERROR("Thrift client could not connect to server");
    }
    return clientIsOpen;
}

// ###############################################################################################
tResult ClassificationServer::Init(tInitStage eStage, __exception) {
    //never miss calling the parent implementation first!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    // update all property values
    PropertyChanged(propNormName);
    PropertyChanged(propHullName);
    PropertyChanged(propModelName);
    PropertyChanged(propThriftAddressName);
    PropertyChanged(propThriftPortName);

    if (eStage == StageFirst) {
        // obstacles output
        cObjectPtr<IMediaType> pOutputType;
        pOutputType = NULL;
        RETURN_IF_FAILED(AllocMediaType(&pOutputType,
                                        MEDIA_TYPE_OBSTACLE_DATA, MEDIA_SUBTYPE_OBSTACLE_DATA,
                                        __exception_ptr));
        RETURN_IF_FAILED(outputPinObstacles.Create("Obstacles_Output", pOutputType,
                                                   this));
        RETURN_IF_FAILED(RegisterPin(&outputPinObstacles));

        // image input
        cObjectPtr<IMediaType> pInputType;
        pInputType = NULL;
        RETURN_IF_FAILED(inputPinImage.Create("Video_Input", IPin::PD_Input,
                                              static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&inputPinImage));

        // obstacles input
        pInputType = NULL;
        RETURN_IF_FAILED(AllocMediaType(&pInputType,
                                        MEDIA_TYPE_OBSTACLE_DATA, MEDIA_SUBTYPE_OBSTACLE_DATA,
                                        __exception_ptr));
        RETURN_IF_FAILED(inputPinObstacles.Create("Obstacle_Input", pInputType, this));
        RETURN_IF_FAILED(RegisterPin(&inputPinObstacles));

        // debug image output
        RETURN_IF_FAILED(outputPinDebugImage.Create("Debug_RGB_out", IPin::PD_Output,
                                                    static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&outputPinDebugImage));
    } else if (eStage == StageNormal) {
        // nothing to do
    } else if (eStage == StageGraphReady) {
        // prepare thrift connection
        boost::shared_ptr<apache::thrift::protocol::TTransport> socket(
                new apache::thrift::transport::TSocket(filterProperties.address.GetPtr(),
                                                       filterProperties.port));
        boost::shared_ptr<apache::thrift::protocol::TTransport>
                transport(new apache::thrift::transport::TBufferedTransport(socket));
        boost::shared_ptr<apache::thrift::protocol::TProtocol>
                protocol(new apache::thrift::protocol::TBinaryProtocol(transport));
        // make the client
        connection.client =
                std::make_shared<thrift_interface::ThriftInterface_ServiceClient>(protocol);

        // create the client thread
        tResult nResult = thread.Create(cKernelThread::TF_Suspended,
                                        static_cast<adtf::IKernelThreadFunc *>(this));

        if (IS_FAILED(nResult)) {
            THROW_ERROR_DESC(nResult, "Failed to create threads");
        }

        // set the image format of the input video pin
        UpdateInputImageFormat();
    }

    RETURN_NOERROR;
}

// ###############################################################################################
tResult ClassificationServer::UpdateInputImageFormat() {
    RETURN_IF_FAILED(updateInputImageFormat(inputPinImage.GetFormat(), inputPinImageFormat,
                                            bufferFront.image));
    RETURN_NOERROR;
}

// ###############################################################################################
tResult ClassificationServer::PropertyChanged(const tChar *strName) {
    RETURN_IF_FAILED(cFilter::PropertyChanged(strName));
    if (strName == propThriftPortName) {
        filterProperties.port = GetPropertyInt(propThriftPortName);
    } else if (strName == propThriftAddressName) {
        filterProperties.address = GetPropertyStr(propThriftAddressName);
    } else if (strName == propModelName) {
        cFilename absoluteFilePath = GetPropertyStr(propModelName);
        ADTF_GET_CONFIG_FILENAME(absoluteFilePath);
        if (absoluteFilePath != filterProperties.model)
        {
            filterProperties.model = absoluteFilePath;
            reloadModel = true;
        }
    } else if (strName == propHullName) {
        filterProperties.useHull = GetPropertyBool(propHullName);
    } else if (strName == propNormName) {
        filterProperties.doNormalize = GetPropertyBool(propNormName);
    }
    RETURN_NOERROR;
}

// ###############################################################################################
// todo load labelmap from python server
// {'car': 0, 'sign': 4, 'pylon': 2, 'gabi': 1, 'steffi': 5, 'road': 3}
static const std::map<const tInt16, const ObstacleType> p_LABEL_TO_TYPE = {
        {-1, NONE},
        {0,  CAR},
        {5,  ADULT},
        {1,  CHILD},
        {4,  SIGN},
        {2,  PYLON},
        {3,  ROAD}
};
static const std::map<const ObstacleType, const tInt16> p_TYPE_TO_LABEL = {
        {NONE,  -1},
        {CAR,   0},
        {ADULT, 5},
        {CHILD, 1},
        {SIGN,  4},
        {PYLON, 2},
        {ROAD,  3}
};

// ###############################################################################################
ObstacleType ClassificationServer::mapLabelToType(tInt16 label) const {
    if (p_LABEL_TO_TYPE.count(label) > 0) {
        return p_LABEL_TO_TYPE.at(label);
    }
    return NONE;
}

// ###############################################################################################
tInt16 ClassificationServer::mapTypeToLabel(const ObstacleType type) const {
    if (p_TYPE_TO_LABEL.count(type) > 0) {
        return p_TYPE_TO_LABEL.at(type);
    }
    return p_TYPE_TO_LABEL.at(NONE);
}

// ###############################################################################################
ObstacleType ClassificationServer::mapStringToType(const std::string &str) const {
    if (OBSTACLE_NAME_TO_TYPE_MAP.count(str) > 0) {
        return OBSTACLE_NAME_TO_TYPE_MAP.at(str);
    }
    return NONE;
}

// ###############################################################################################
std::string ClassificationServer::mapTypeToString(const ObstacleType type) const {
    if (OBSTACLE_TYPE_TO_NAME_MAP.count(type) > 0) {
        return OBSTACLE_TYPE_TO_NAME_MAP.at(type);
    }
    return OBSTACLE_TYPE_TO_NAME_MAP.at(NONE);

}

// ###############################################################################################
tInt16 ClassificationServer::mapStringToLabel(const std::string &str) const {
    return mapTypeToLabel(mapStringToType(str));
}

// ###############################################################################################
std::string ClassificationServer::mapLabelToString(const tInt16 label) const {
    return mapTypeToString(mapLabelToType(label));
}

// ###############################################################################################

