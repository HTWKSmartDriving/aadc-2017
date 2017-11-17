#include "LaneDetectionFilter.h"

ADTF_FILTER_PLUGIN(OID_ADTF_FILTER_DEF, OID_ADTF_FILTER_DEF, LaneDetectionFilter)

LaneDetectionFilter::LaneDetectionFilter(const tChar *__info) : cFilter(__info) {
    SetPropertyInt("Start Row", PROPERTY_START_ROW);
    SetPropertyStr("Start Row" NSSUBPROP_DESCRIPTION, "Start row for detection");
    SetPropertyInt("Start Row" NSSUBPROP_MIN, PROPERTY_MIN);
    SetPropertyInt("Start Row" NSSUBPROP_MAX, PROPERTY_MAX_HEIGHT);

    SetPropertyInt("Stop Row", PROPERTY_STOP_ROW);
    SetPropertyStr("Stop Row" NSSUBPROP_DESCRIPTION, "Stop row for detection");
    SetPropertyInt("Stop Row" NSSUBPROP_MIN, PROPERTY_MIN);
    SetPropertyInt("Stop Row" NSSUBPROP_MAX, PROPERTY_MAX_HEIGHT);

    SetPropertyInt("Step Width", PROPERTY_STEP_WIDTH);
    SetPropertyStr("Step Width" NSSUBPROP_DESCRIPTION, "Steps between start and stop");
    SetPropertyInt("Step Width" NSSUBPROP_MIN, PROPERTY_MIN);
    SetPropertyInt("Step Width" NSSUBPROP_MAX, PROPERTY_MAX_HEIGHT);

    SetPropertyInt("Frame Count", PROPERTY_FRAME_COUNT);
    SetPropertyStr("Frame Count" NSSUBPROP_DESCRIPTION, "Analyze count of frames for detection");
    SetPropertyInt("Frame Count" NSSUBPROP_MIN, PROPERTY_MIN);
    SetPropertyInt("Frame Count" NSSUBPROP_MAX, PROPERTY_MAX_FRAMES);

    SetPropertyBool("Sort Bottom Up", tTrue);
    SetPropertyStr("Sort Bottom Up" NSSUBPROP_DESCRIPTION, "Sets scanline ordering");

    std::stringstream valueListStringBuilder;
    valueListStringBuilder << YEN << "@" << "YEN|" << YEN_OTSU << "@" << "YEN_OTSU|"
                           << YEN_TRIANGLE << "@" << "YEN_TRIANGLE|" << ADAPTIVE_MEAN_C << "@" << "ADAPTIVE_MEAN_C|"
                           << ADAPTIVE_GAUSSIAN_C << "@" << "ADAPTIVE_GAUSSIAN_C";

    SetPropertyInt("Threshold Algorithm", YEN);
    SetPropertyBool("Threshold Algorithm" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyStr("Threshold Algorithm" NSSUBPROP_VALUELIST, valueListStringBuilder.str().c_str());
    SetPropertyStr("Threshold Algorithm" NSSUBPROP_DESCRIPTION,
                   "Defines method which is used to create the output image");
}

tResult LaneDetectionFilter::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid **) &pDescManager, __exception_ptr));

        // Video Input
        RETURN_IF_FAILED(videoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&videoInputPin));

        //TODO MediaType durch Description Manager?
        // get a media type for the output pin
        cObjectPtr<IMediaType> pOutputType;
        RETURN_IF_FAILED(AllocMediaType(&pOutputType,
                                        MEDIA_TYPE_CAMERA_RESOLUTION_DATA, MEDIA_SUBTYPE_CAMERA_RESOLUTION_DATA,
                                        __exception_ptr));

        // create and register output pin
        RETURN_IF_FAILED(outputCameraResolutionDataPin.Create("Camera_Resolution_Data", pOutputType, this));
        RETURN_IF_FAILED(RegisterPin(&outputCameraResolutionDataPin));
        pOutputType = NULL;

        RETURN_IF_FAILED(AllocMediaType(&pOutputType, MEDIA_TYPE_LANE_DATA, MEDIA_SUBTYPE_LANE_DATA, __exception_ptr));

        // create and register the output pin
        RETURN_IF_FAILED(outputLanePointsPin.Create("Lane_Detect_Result", pOutputType, this));
        RETURN_IF_FAILED(RegisterPin(&outputLanePointsPin));

        // create and register debug video output pin
        RETURN_IF_FAILED(
                videoOutputPin.Create("Debug_Video_Output", IPin::PD_Output, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&videoOutputPin));

    } else if (eStage == StageGraphReady) {
        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(videoInputPin.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid **) &pTypeVideo));

        // set the image format of the input video pin
        updateInputImageFormat(pTypeVideo->GetFormat(), inputBitmapFormat, inputImage);
        RETURN_IF_FAILED(thread.Create(NULL, 0, cThread::TS_Suspended, static_cast<IThreadFunc *>(this)));
    }
    RETURN_NOERROR;
}

tResult LaneDetectionFilter::Stop(ucom::IException **__exception_ptr) {
    if (thread.GetState() == cThread::TS_Running) {
        thread.Suspend(tTrue);
    }
    return cFilter::Stop(__exception_ptr);
}

tResult LaneDetectionFilter::Shutdown(tInitStage eStage, __exception) {
    if (eStage == StageNormal) {
        thread.Terminate(tTrue);
        thread.Release();
    }
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult LaneDetectionFilter::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
                                        IMediaSample *pMediaSample) {
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        if (pSource == &videoInputPin) {
            if (!detector) {
                updateInputImageFormat(videoInputPin.GetFormat(), inputBitmapFormat, inputImage);
                RETURN_IF_FAILED(transmitCameraHeightAndWidth(pMediaSample));
                initLaneDetection();
            }
            if(thread.GetState() == cThread::TS_Suspended){
                time = pMediaSample->GetTime();
                RETURN_IF_FAILED(bufferToMat(videoInputPin.GetFormat(), pMediaSample, inputImage, inputBitmapFormat));
                RETURN_IF_FAILED(thread.Run())
            }
        }
    } else if (nEventCode == IPinEventSink::PE_MediaTypeChanged) {
        if (pSource == &videoInputPin) {
            //the input format was changed, so the imageformat has to changed in this filter also
            updateInputImageFormat(videoInputPin.GetFormat(), inputBitmapFormat, inputImage);
            //transmit only once: camera resolution information over pin
            RETURN_IF_FAILED(transmitCameraHeightAndWidth(pMediaSample));
            initLaneDetection();
        }
    }
    RETURN_NOERROR;
}

/**
 * On First call of OnPinEvent, after pictureSize is known the detector should be constructed
 */
void LaneDetectionFilter::initLaneDetection() {
    int checkHeight = inputBitmapFormat.nHeight - 1; //including 0

    int start = GetPropertyInt("Start Row");
    if (start > checkHeight) {
        LOG_WARNING(adtf_util::cString::Format("Start Row to large %d is set to %d", start, checkHeight));
        start = checkHeight;
    }
    int stop = GetPropertyInt("Stop Row");
    if (stop > checkHeight) {
        LOG_WARNING(adtf_util::cString::Format("Stop Row to large %d is set to %d", stop, checkHeight));
        stop = checkHeight;
    }
    const int step = GetPropertyInt("Step Width");
    const int frames = GetPropertyInt("Frame Count");
    const bool sortBottomUp = GetPropertyBool("Sort Bottom Up");
    ThresholdAlgorithm algorithm = (ThresholdAlgorithm) GetPropertyInt("Threshold Algorithm");
    //LOG_INFO(adtf_util::cString::Format("LaneDetection: Start %d, Stop %d, Step %d, Frames %d, BottomUp %d, Method %d",
    //                                    start, stop, step, frames, sortBottomUp, algorithm));

    detector = unique_ptr<SpatioTemporalLaneDetector>
            (new SpatioTemporalLaneDetector(start, stop, step, frames, sortBottomUp, algorithm));
}

tResult LaneDetectionFilter::ThreadFunc(cThread *Thread, tVoid *data, tSize size){
    if (!inputImage.empty()) {
        //std::vector<tLanePoint> results = = detector.detectLanes(inputImage);
        std::vector<tLanePoint> results = detector->parallelDetectLanes(inputImage);
        RETURN_IF_FAILED(transmitResult(results));
        if (videoOutputPin.IsConnected()) {
            RETURN_IF_FAILED(processDebugVideo(inputImage, results));
        }
    }
    RETURN_IF_FAILED(thread.Suspend());
    RETURN_NOERROR;
}

tResult LaneDetectionFilter::transmitResult(vector<tLanePoint> &results) {
    if (results.empty()) {
        RETURN_NOERROR;
    }
    cObjectPtr<IMediaSample> pOutputLanePoints;
    if (IS_OK(AllocMediaSample(&pOutputLanePoints))) {
        pOutputLanePoints->Update(time, &results.front(), sizeof(tLanePoint) * results.size(), 0);
        RETURN_IF_FAILED(outputLanePointsPin.Transmit(pOutputLanePoints));
    }
    RETURN_NOERROR;
}

/**
 *+ send informations about Camera Height and Width
**/
tResult LaneDetectionFilter::transmitCameraHeightAndWidth(const IMediaSample *pMediaSample) {
    cObjectPtr<IMediaSample> pOutputResolutionData;
    if (IS_OK(AllocMediaSample(&pOutputResolutionData))) {
        tCameraResolutionData resolutionData;
        resolutionData.width = inputBitmapFormat.nWidth;
        resolutionData.height = inputBitmapFormat.nHeight;
        pOutputResolutionData->Update(pMediaSample->GetTime(), &resolutionData, sizeof(resolutionData), 0);
        RETURN_IF_FAILED(outputCameraResolutionDataPin.Transmit(pOutputResolutionData));
    }
    RETURN_NOERROR;
}

/**
 * Shows debug Video on outputPin with laneResults
 */
tResult LaneDetectionFilter::processDebugVideo(Mat &image, const std::vector<tLanePoint> &result) {

    debugLaneDetection(image, result);
    updateOutputImageFormat(image, outputBitmapFormat, videoOutputPin);

    cImage newImage;
    newImage.Create(outputBitmapFormat.nWidth, outputBitmapFormat.nHeight, outputBitmapFormat.nBitsPerPixel,
                    outputBitmapFormat.nBytesPerLine, image.data);

    //create the new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));
    //updating media sample
    RETURN_IF_FAILED(pMediaSample->Update(time, newImage.GetBitmap(), newImage.GetSize(),
                                          IMediaSample::MSF_None));
    //transmitting
    RETURN_IF_FAILED(videoOutputPin.Transmit(pMediaSample));
    RETURN_NOERROR;
}

/**
 * only for debug purpose, use defines to enable/disable
 */
inline void LaneDetectionFilter::debugLaneDetection(Mat &workingImage, const std::vector<tLanePoint> &result) {
    for (unsigned int i = 0; i < result.size(); i++) {
        Point left = Point(result[i].xLeftLane, result[i].y);
        bool leftGuessed = result[i].isLeftGuessed;
        Point right = Point(result[i].xRightLane, result[i].y);
        bool rightGuessed = result[i].isRightGuessed;

        if (leftGuessed) {
            circle(workingImage, left, 5, CV_RGB(255, 255, 255), -1);
        } else {
            circle(workingImage, left, 5, CV_RGB(255, 0, 0), -1);
        }
        if (rightGuessed) {
            circle(workingImage, right, 5, CV_RGB(255, 255, 255), -1);
        } else {
            circle(workingImage, right, 5, CV_RGB(0, 0, 255), -1);
        }
    }

    for (unsigned int i = 0; i < result.size(); i++) {
        Point left = Point(result[i].xLeftLane, result[i].y);
        bool leftGuessed = result[i].isLeftGuessed;
        Point right = Point(result[i].xRightLane, result[i].y);
        bool rightGuessed = result[i].isRightGuessed;

        if (leftGuessed || rightGuessed) {
        } else {
            circle(workingImage, left, 5, CV_RGB(255, 0, 0), -1);
            circle(workingImage, right, 5, CV_RGB(0, 0, 255), -1);

            line(workingImage, left, right, CV_RGB(0, 0, 0));
            Point mid = Point((left.x + right.x) / 2, left.y);
            circle(workingImage, mid, 3, CV_RGB(0, 0, 0), -1);

            Size imageSize = workingImage.size();
            Point center(imageSize.width / 2, imageSize.height / 2);
            Point ref(imageSize.width / 2, imageSize.height);

            circle(workingImage, center, 3, CV_RGB(0, 0, 0), -1);
            circle(workingImage, ref, 3, CV_RGB(0, 0, 0), -1);
            putText(workingImage, "R", ref, CV_FONT_HERSHEY_COMPLEX, 1.5, CV_RGB(0, 0, 0), 1, CV_AA);
            putText(workingImage, "C", center, CV_FONT_HERSHEY_COMPLEX, 1.5, CV_RGB(0, 0, 0), 1, CV_AA);
            putText(workingImage, "M", mid, CV_FONT_HERSHEY_COMPLEX, 1.5, CV_RGB(0, 0, 0), 1, CV_AA);
            line(workingImage, center, ref, CV_RGB(0, 0, 0));
            line(workingImage, mid, ref, CV_RGB(0, 0, 0));
            break;
        }
    }
}
