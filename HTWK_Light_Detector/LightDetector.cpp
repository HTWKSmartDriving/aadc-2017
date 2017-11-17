#include "LightDetector.h"

ADTF_FILTER_PLUGIN(OID_ADTF_FILTER_DEF, OID_ADTF_FILTER_DEF, LightDetector)

LightDetector::LightDetector(const tChar *__info) : cFilter(__info) {
    SetPropertyBool(PROPERTY_FLIP_HORIZONTAL, tFalse);
    SetPropertyBool(PROPERTY_FLIP_VERTICAL, tFalse);
}

tResult LightDetector::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid **) &pDescManager, __exception_ptr));

        RETURN_IF_FAILED(videoInputPin.Create("RearCamVideo", IPin::PD_Input, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&videoInputPin));

        RETURN_IF_FAILED(
                videoDebugPin.Create("DebugVideo", IPin::PD_Output, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&videoDebugPin));

        RETURN_IF_FAILED(
                videoOriginalPin.Create("VideoCropped", IPin::PD_Output, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&videoOriginalPin));

        cObjectPtr<IMediaType> pOutputType;
        RETURN_IF_FAILED(
                AllocMediaType(&pOutputType, MEDIA_TYPE_OBSTACLE_DATA, MEDIA_SUBTYPE_OBSTACLE_DATA, __exception_ptr));
        RETURN_IF_FAILED(obstacleOutputPin.Create("Obstacles", pOutputType, this));
        RETURN_IF_FAILED(RegisterPin(&obstacleOutputPin));

    } else if (eStage == StageGraphReady) {
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(videoInputPin.GetMediaType(&pType));
        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid **) &pTypeVideo));
        updateInputImageFormat(pTypeVideo->GetFormat(), inputBitmapFormat, inputImage);

        RETURN_IF_FAILED(thread.Create(NULL, 0, cThread::TS_Suspended, static_cast<IThreadFunc *>(this)));
    }
    RETURN_NOERROR;
}

tResult LightDetector::Start(ucom::IException **__exception_ptr) {
    bool fV = GetPropertyBool(PROPERTY_FLIP_VERTICAL);
    bool fH = GetPropertyBool(PROPERTY_FLIP_HORIZONTAL);
    if (fV && fH) {
        flipper = -1; //both
    } else if (fV && !fH) {
        flipper = 1; //flip vertical
    } else if (fH && !fV) {
        flipper = 0; // flip horizontal
    } else {
        flipper = INT_MAX; //dont flip
    }
    id = 0;
    return cFilter::Start(__exception_ptr);
}

tResult LightDetector::Stop(ucom::IException **__exception_ptr) {
    if (thread.GetState() == cThread::TS_Running) {
        thread.Suspend(tTrue);
    }
    return cFilter::Stop(__exception_ptr);
}

tResult LightDetector::Shutdown(tInitStage eStage, __exception) {
    if (eStage == StageNormal) {
        thread.Terminate(tTrue);
        thread.Release();
    }
    return cFilter::Shutdown(eStage, __exception_ptr);
}


tResult LightDetector::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
                                  IMediaSample *pMediaSample) {
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        if (pSource == &videoInputPin && thread.GetState() == cThread::TS_Suspended) {
            bufferToMat(videoInputPin.GetFormat(), pMediaSample, inputImage, inputBitmapFormat);
            time = pMediaSample->GetTime();
            RETURN_IF_FAILED(thread.Run());
        }
    } else if (nEventCode == IPinEventSink::PE_MediaTypeChanged) {
        if (pSource == &videoInputPin) {
            updateInputImageFormat(videoInputPin.GetFormat(), inputBitmapFormat, inputImage);
        }
    }
    RETURN_NOERROR;
}

tResult LightDetector::ThreadFunc(cThread *Thread, tVoid *data, tSize size) {
    if (inputImage.rows != defaultSize.height || inputImage.cols != defaultSize.width) {
        //Size doesnt match, resize to necessary size
        resize(inputImage, croppedImage, defaultSize);
        //Flip resized image
        if (flipper != INT_MAX) {
            flip(croppedImage, croppedImage, flipper);
        }
        croppedImage = croppedImage(crop).clone();
    } else if (flipper != INT_MAX) {
        //Flip not resized image
        flip(inputImage, inputImage, flipper);
        croppedImage = inputImage(crop).clone();
    } else {
        //no flip, no resize
        croppedImage = inputImage(crop).clone();
    }

    initalize();
    resultRect.clear();

    tryToFilterCars();
    findLight();

    if (obstacleOutputPin.IsConnected()) {
        vector<tObstacleData> police;
        for (const auto &res: resultRect) {
            tObstacleData data;
            data.rgbROI.topLeft = res.tl();
            data.rgbROI.bottomRight = res.br();
            data.id = id++;
            data.radius = 0;
            data.height = 0;
            data.obstacle = Point2f(0, 0);
            data.distance = 0;
            data.status = ObstacleStatus::REARCAM;
            police.emplace_back(data);
        }
        if (!police.empty()) {
            transmitVideo(croppedImage, videoOriginalPin, outputOriginalFormat);
            transmitObstacles(police);
        }
    }

    if (videoDebugPin.IsConnected()) {
        for (const auto &bound : possibleBoundRect) {
            rectangle(croppedImage, bound.tl(), bound.br(), Scalar(0, 0, 255));
        }
        drawKeypoints(croppedImage, lightDetectionResult, croppedImage, Scalar(0, 255, 0),
                      DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        transmitVideo(croppedImage, videoDebugPin, debugBitmapFormat);
    }
    RETURN_IF_FAILED(thread.Suspend());
    RETURN_NOERROR;
}

void LightDetector::initalize() {
    if (!init) {
        params.minThreshold = 240;
        params.maxThreshold = 255;
        params.filterByArea = true;
        params.minArea = 3;
        params.maxArea = 100;
        params.filterByCircularity = true;
        params.minCircularity = 0.685; // 0.785 is rectangle
        params.maxCircularity = 0.885;
        params.filterByConvexity = false;
        params.minConvexity = 0.87;
        params.filterByInertia = false;
        params.minInertiaRatio = 0.01;
        params.filterByColor = false;
        detector = SimpleBlobDetector::create(params);
        init = true;
    }
}

void LightDetector::findLight() {
    lightDetectionResult.clear();
    for (auto &possible : possibleBoundRect) {
        Point2f startPos;
        auto val = possible.x - includeRoiBorder;
        startPos.x = val < 0 ? 0 : val;
        val = possible.y - possible.height - includeRoiBorder;
        startPos.y = val < 0 ? 0 : val;
        val = possible.width + includeRoiBorder + includeRoiBorder;
        const auto w = croppedImage.cols - startPos.x - includeRoiBorder <= val ?
                       croppedImage.cols - startPos.x - includeRoiBorder : val;
        val = possible.height + includeRoiBorder + includeRoiBorder;
        const auto h = croppedImage.cols - startPos.x - includeRoiBorder <= val ?
                       croppedImage.cols - startPos.x - includeRoiBorder : val;

        Mat workingImage;
        absdiff(croppedImage(Rect(startPos.x, startPos.y, w, h)), Scalar(255, 255, 255), workingImage);
        Mat bgr[3];
        split(workingImage, bgr);

        bitwise_not(bgr[0], bgr[0]);
        bitwise_not(bgr[2], bgr[2]);

        threshold(bgr[0], bgr[0], 240, 255, THRESH_BINARY);
        threshold(bgr[2], bgr[2], 200, 255, THRESH_BINARY);
        bitwise_xor(bgr[0], bgr[2], workingImage);
        morphologyEx(workingImage, workingImage, MORPH_CLOSE, structuringElement);
        bitwise_not(workingImage, workingImage);
        vector<KeyPoint> keypoints;
        detector->detect(workingImage, keypoints);
        if (keypoints.size() >= 2) {
            bool foundLight = false;
            for (KeyPoint &key : keypoints) {
                for (KeyPoint &check : keypoints) {
                    if (key.pt == check.pt) {
                        continue;
                    }
                    const auto boundCheck = Point2f(abs(check.pt.x - key.pt.x), abs(check.pt.y - key.pt.y));
                    //Doen't jump to hard
                    if (boundCheck.x < 50 && boundCheck.y < 15) {
                        KeyPoint pt = key;
                        pt.pt += startPos;
                        lightDetectionResult.emplace_back(pt);
                        foundLight = true;
                        break;
                    }
                }
            }
            if (foundLight) {
                resultRect.emplace_back(possible);
            }
        }
    }
}

void LightDetector::tryToFilterCars() {
    possibleBoundRect.clear();
    Mat bgr[3];
    split(croppedImage, bgr);
    //Apply thresholding
    threshold(bgr[0], bgr[0], 135, 255, THRESH_BINARY);
    threshold(bgr[1], bgr[1], 135, 255, THRESH_BINARY);
    threshold(bgr[2], bgr[2], 135, 255, THRESH_BINARY);

    bgr[0] -= bgr[1];
    bgr[0] -= bgr[2];

    bgr[2] -= bgr[0];
    bgr[2] -= bgr[1];

    morphologyEx(bgr[0], bgr[0], MORPH_OPEN, structuringElement);
    morphologyEx(bgr[0], bgr[0], MORPH_CLOSE, structuringElement);
    extractPossibleCarRects(bgr[0]);
}

void LightDetector::extractPossibleCarRects(const Mat &binaryBlueChannel) {
    vector<vector<Point>> contours;
    findContours(binaryBlueChannel, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    vector<vector<Point>> contours_poly(contours.size());
    for (size_t i = 0; i < contours.size(); ++i) {
        approxPolyDP(contours[i], contours_poly[i], 3, true);

        Rect tmp = boundingRect(contours_poly[i]);
        //maximal 1/4 of picture
        if (tmp.area() > minAreaSizeCar && tmp.area() < static_cast<int>(croppedImage.total() / 3)) {
            bool isMerged = false;
            for (auto &rect : possibleBoundRect) {
                if ((rect & tmp) == rect) { //rect is in tmp
                    rect = tmp;
                    isMerged = true;
                } else if ((rect & tmp) == tmp) { //tmp is in rect
                    isMerged = true;
                } else if ((rect & tmp).area() > 0) {// they intersect; merge them.
                    rect = rect | tmp;
                    isMerged = true;
                }
            }
            if (!isMerged) {
                possibleBoundRect.emplace_back(tmp);
            }
        }
    }
}

tResult LightDetector::transmitVideo(Mat &image, cVideoPin &pin, tBitmapFormat &bmp) {
    if (pin.IsConnected()) {
        updateOutputImageFormat(image, bmp, pin);
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));
        RETURN_IF_FAILED(pMediaSample->AllocBuffer(bmp.nSize));
        RETURN_IF_FAILED(pMediaSample->Update(time, image.data, bmp.nSize, IMediaSample::MSF_None));
        RETURN_IF_FAILED(pin.Transmit(pMediaSample));
    }
    RETURN_NOERROR;
}

void LightDetector::transmitObstacles(const vector<tObstacleData> &obstacles) {
    if (obstacleOutputPin.IsConnected() && !obstacles.empty()) {
        cObjectPtr<IMediaSample> pOutputObstacleMap;
        if (IS_OK(AllocMediaSample(&pOutputObstacleMap))) {
            pOutputObstacleMap->Update(time, &obstacles.front(),
                                       sizeof(tObstacleData) * obstacles.size(), 0);
            obstacleOutputPin.Transmit(pOutputObstacleMap);
        }
    }
}



