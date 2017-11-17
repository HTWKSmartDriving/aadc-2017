#include "PreObstacleFilter.h"

ADTF_FILTER_PLUGIN(OID_ADTF_FILTER_DEF, OID_ADTF_FILTER_DEF, PreObstacleFilter)

PreObstacleFilter::PreObstacleFilter(const tChar *__info) : cFilter(__info) {
    SetPropertyFloat(PROPERTY_TIME_OUT, PROPERTY_DEFAULT_TIME_OUT);
    SetPropertyStr(PROPERTY_TIME_OUT NSSUBPROP_DESCRIPTION, "Timeout in s for waiting for an inital sign list");
    SetPropertyFloat(PROPERTY_TIME_OUT  NSSUBPROP_MIN, 0.0);
    SetPropertyFloat(PROPERTY_TIME_OUT  NSSUBPROP_MAX, 30.0);

    SetPropertyInt(PROPERTY_THRESHOLD, PROPERTY_DEFAULT_THRESHOLD);
    SetPropertyStr(PROPERTY_THRESHOLD  NSSUBPROP_DESCRIPTION, "Sets Threshold for Road Filtering");
    SetPropertyInt(PROPERTY_THRESHOLD  NSSUBPROP_MIN, 1);
    SetPropertyInt(PROPERTY_THRESHOLD  NSSUBPROP_MAX, 200);

    SetPropertyFloat(PROPERTY_VOTE_NOT_ROAD, PROPERTY_DEFAULT_VOTE_NOT_ROAD);
    SetPropertyStr(PROPERTY_VOTE_NOT_ROAD NSSUBPROP_DESCRIPTION,
                   "E.g. if binarization img contains more then 33 % white, then this is voted as non road");
    SetPropertyFloat(PROPERTY_VOTE_NOT_ROAD  NSSUBPROP_MIN, 0.0);
    SetPropertyFloat(PROPERTY_VOTE_NOT_ROAD  NSSUBPROP_MAX, 0.75);

    SetPropertyFloat(PROPERTY_SIGN_RADIUS, PROPERTY_DEFAULT_SIGN_RADIUS);
    SetPropertyStr(PROPERTY_SIGN_RADIUS  NSSUBPROP_DESCRIPTION, "Sets Sign Radius");
    SetPropertyFloat(PROPERTY_SIGN_RADIUS  NSSUBPROP_MIN, 0.0);
    SetPropertyFloat(PROPERTY_SIGN_RADIUS  NSSUBPROP_MAX, 1.0);

    SetPropertyBool(PROPERTY_FILTER_ROAD, PROPERTY_DEFAULT_FILTER_ROAD);
    SetPropertyStr(PROPERTY_FILTER_ROAD NSSUBPROP_DESCRIPTION, "Enables or Disables Road Filtering");

    //SetPropertyBool(PROPERTY_COLOR_EQUALIZE, PROPERTY_DEFAULT_COLOR_EQUALIZE);
    //SetPropertyStr(PROPERTY_COLOR_EQUALIZE NSSUBPROP_DESCRIPTION, "Enables or Disables Equalization of color");
}

tResult PreObstacleFilter::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {
        cObjectPtr<IMediaType> pInputType;
        RETURN_IF_FAILED(
                AllocMediaType(&pInputType, MEDIA_TYPE_INITAL_ROAD_SIGN_DATA, MEDIA_SUBTYPE_INITAL_ROAD_SIGN_DATA,
                               __exception_ptr));
        RETURN_IF_FAILED(inputInitalRoadSignsPin.Create("InitalRoadSigns", pInputType, this));
        RETURN_IF_FAILED(RegisterPin(&inputInitalRoadSignsPin));

        pInputType = NULL;
        RETURN_IF_FAILED(AllocMediaType(&pInputType, MEDIA_TYPE_OBSTACLE_DATA, MEDIA_SUBTYPE_OBSTACLE_DATA,
                                        __exception_ptr));
        // create and register the input pin
        RETURN_IF_FAILED(obstacleInputPin.Create("Obstacles", pInputType, this));
        RETURN_IF_FAILED(RegisterPin(&obstacleInputPin));
        RETURN_IF_FAILED(
                rgbInputPin.Create("RGB", IPin::PD_Input, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&rgbInputPin));

        cObjectPtr<IMediaType> pOutputType;
        RETURN_IF_FAILED(
                AllocMediaType(&pOutputType, MEDIA_TYPE_OBSTACLE_DATA, MEDIA_SUBTYPE_OBSTACLE_DATA, __exception_ptr));
        RETURN_IF_FAILED(obstacleOutputPin.Create("Updated_Obstacles", pOutputType, this));
        RETURN_IF_FAILED(RegisterPin(&obstacleOutputPin));

        RETURN_IF_FAILED(
                rgbOutputPin.Create("RGB_Output", IPin::PD_Output, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&rgbOutputPin));

    } else if (eStage == StageGraphReady) {
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(rgbInputPin.GetMediaType(&pType));
        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid **) &pTypeVideo));
        updateInputImageFormat(pTypeVideo->GetFormat(), rgbInputBitmapFormat, rgbImage);
    }
    RETURN_NOERROR;
}

tResult PreObstacleFilter::Start(ucom::IException **__exception_ptr) {
    imageUpdated = false;
    obstaclesUpdated = false;
    signsUpdated = false;
    threshold = GetPropertyInt(PROPERTY_THRESHOLD);
    signRadius = static_cast<float>(GetPropertyFloat(PROPERTY_SIGN_RADIUS));
    voteToNotRoad = static_cast<float>(GetPropertyFloat(PROPERTY_VOTE_NOT_ROAD));
    timeOutWaitSign = static_cast<float>(GetPropertyFloat(PROPERTY_TIME_OUT));
    isRoadFilteringEnabled = static_cast<bool>(GetPropertyBool(PROPERTY_FILTER_ROAD));
    //isColorEqualizationEnabled = static_cast<bool>(GetPropertyBool(PROPERTY_COLOR_EQUALIZE));
    startWaitTimeSign = _clock->GetTime();
    return cFilter::Start(__exception_ptr);
}

tResult PreObstacleFilter::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
                                      IMediaSample *pMediaSample) {
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        if (!imageUpdated && pSource == &rgbInputPin) {
            RETURN_IF_FAILED(
                    bufferToMat(rgbInputPin.GetFormat(), pMediaSample, rgbImage, rgbInputBitmapFormat));
            imageUpdated = true;
        } else if (!obstaclesUpdated && pSource == &obstacleInputPin) {
            htwk::recieveVector(pMediaSample, recObstacleMap);
            obstaclesUpdated = true;
        } else if (pSource == &inputInitalRoadSignsPin) {
            signMap.clear();
            {
                __adtf_sample_read_lock(pMediaSample, tInitialRoadSign, pData);
                for (size_t i = 0; i < pMediaSample->GetSize() / sizeof(tInitialRoadSign); ++i) {
                    signMap.push_back(pData[i].pos);
                }
                signsUpdated = true;
            }
        }
        if (!signsUpdated && imageUpdated && obstaclesUpdated) {
            //Timeout for waiting for inital Signs
            if (static_cast<tFloat32>((_clock->GetTime() - startWaitTimeSign) * 1e-6) >= timeOutWaitSign) {
                signsUpdated = true;
            }
            //drop samples
            imageUpdated = false;
            obstaclesUpdated = false;
        } else if (imageUpdated && obstaclesUpdated) {
            filterObstacleMap(recObstacleMap, pMediaSample->GetTime());
            imageUpdated = false;
            obstaclesUpdated = false;
        }
    } else if (nEventCode == IPinEventSink::PE_MediaTypeChanged && (pSource == &rgbInputPin)) {
        RETURN_IF_FAILED(updateInputImageFormat(rgbInputPin.GetFormat(), rgbInputBitmapFormat, rgbImage));
    }
    RETURN_NOERROR;
}

void PreObstacleFilter::filterObstacleMap(const vector<tObstacleData> &inputVec, const tTimeStamp &time) {
    //filter Signs aus inputVec
    vector<tObstacleData> filteredInput;
    for (auto &in : inputVec) {
        bool isSign = false;
        for (auto &pos : signMap) {
            const auto rel = htwk::relationPoints::getRelationOf2Points(in.obstacle, pos, in.radius, signRadius);
            if (rel != htwk::relationPoints::NO_MATCH) {
                isSign = true;
                break;
            }
        }
        if (!isSign) {
            if (isRoadFilteringEnabled) {
                //filter Road aus inputVec, wenn aktiviert
                const Rect rect(in.rgbROI.topLeft, in.rgbROI.bottomRight);
                UMat roi;
                cvtColor(rgbImage(rect), roi, COLOR_BGR2GRAY);
                cv::threshold(roi, roi, threshold, 255, cv::THRESH_BINARY);
                //Überprüfe ob mehr als e.g. 2/3 schwarz
                const auto val = countNonZero(roi);
                if (val >= (roi.rows * roi.cols * voteToNotRoad)) {
                    // No road and No Sign, so add
                    filteredInput.emplace_back(in);
                }
            } else {
                filteredInput.emplace_back(in);
            }
        }
    }
    if (!filteredInput.empty()) {
        /*if (isColorEqualizationEnabled) {
            for (const auto &in : filteredInput) {
                Mat roi = rgbImage(Rect(in.rgbROI.topLeft, in.rgbROI.bottomRight));
                vector<Mat> channels(3);
                cv::split(roi, channels);
                equalizeHist(channels[0], channels[0]);
                equalizeHist(channels[1], channels[1]);
                equalizeHist(channels[2], channels[2]);
                cv::merge(channels, roi);
            }
        }*/
        transmitVideo(rgbImage, rgbOutputPin, rgbOutputBitmapFormat, time);
        transmitFilteredObstacles(filteredInput, time);
    }
}

void
PreObstacleFilter::transmitFilteredObstacles(const vector<tObstacleData> &obstacles, const tTimeStamp &time) {
    if (obstacleOutputPin.IsConnected()) {
        cObjectPtr<IMediaSample> pOutputObstacleMap;
        if (IS_OK(AllocMediaSample(&pOutputObstacleMap))) {
            pOutputObstacleMap->Update(time, &obstacles.front(),
                                       sizeof(tObstacleData) * obstacles.size(), 0);
            obstacleOutputPin.Transmit(pOutputObstacleMap);
        }
    }
}

tResult PreObstacleFilter::transmitVideo(const Mat &image, cVideoPin &pin, tBitmapFormat &bmp, const tTimeStamp &time) {
    updateOutputImageFormat(image, bmp, pin);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(bmp.nSize));
    RETURN_IF_FAILED(pMediaSample->Update(time, image.data, bmp.nSize, IMediaSample::MSF_None));
    RETURN_IF_FAILED(pin.Transmit(pMediaSample));
    RETURN_NOERROR;
}