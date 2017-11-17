#include "StixelsToRoisFilter.h"

ADTF_FILTER_PLUGIN(OID_ADTF_FILTER_DEF, OID_ADTF_FILTER_DEF, StixelsToRoisFilter)

StixelsToRoisFilter::StixelsToRoisFilter(const tChar *__info) : cFilter(__info) {
    SetPropertyInt(PROPERTY_FAILURE_DISPARITY, failureDisp);
    SetPropertyStr(PROPERTY_FAILURE_DISPARITY  NSSUBPROP_DESCRIPTION, "Sets failure disparity");
    SetPropertyInt(PROPERTY_FAILURE_DISPARITY  NSSUBPROP_MIN, -2);
    SetPropertyInt(PROPERTY_FAILURE_DISPARITY  NSSUBPROP_MAX, 0);
    SetPropertyBool(PROPERTY_FAILURE_DISPARITY  NSSUBPROP_HIDDEN, tTrue);

    SetPropertyFloat(PROPERTY_MAX_HEIGHT, maxHeight);
    SetPropertyStr(PROPERTY_MAX_HEIGHT  NSSUBPROP_DESCRIPTION, "Maximum Height in Metern");
    SetPropertyFloat(PROPERTY_MAX_HEIGHT  NSSUBPROP_MIN, 0);
    SetPropertyFloat(PROPERTY_MAX_HEIGHT  NSSUBPROP_MAX, 1000);

    SetPropertyFloat(PROPERTY_MIN_HEIGHT, minHeight);
    SetPropertyStr(PROPERTY_MIN_HEIGHT  NSSUBPROP_DESCRIPTION, "Minimum Height in Metern");
    SetPropertyFloat(PROPERTY_MIN_HEIGHT  NSSUBPROP_MIN, 0);
    SetPropertyFloat(PROPERTY_MIN_HEIGHT  NSSUBPROP_MAX, 1000);

    SetPropertyFloat(PROPERTY_MAX_HEIGHT_DIFF, maxHeightDiff);
    SetPropertyStr(PROPERTY_MAX_HEIGHT_DIFF  NSSUBPROP_DESCRIPTION,
                   "Maximum Height Difference Between Neighbours in Metern");
    SetPropertyFloat(PROPERTY_MAX_HEIGHT_DIFF  NSSUBPROP_MIN, 0);
    SetPropertyFloat(PROPERTY_MAX_HEIGHT_DIFF  NSSUBPROP_MAX, 1);

    SetPropertyFloat(PROPERTY_MIN_DISTANCE, PROPERTY_DEFAULT_MIN_DISTANCE);
    SetPropertyStr(PROPERTY_MIN_DISTANCE  NSSUBPROP_DESCRIPTION, "Sets Minimum Distance for check in Metern");
    SetPropertyFloat(PROPERTY_MIN_DISTANCE  NSSUBPROP_MIN, 0);
    SetPropertyFloat(PROPERTY_MIN_DISTANCE  NSSUBPROP_MAX, 10);

    SetPropertyFloat(PROPERTY_MAX_DISTANCE, PROPERTY_DEFAULT_MAX_DISTANCE);
    SetPropertyStr(PROPERTY_MAX_DISTANCE  NSSUBPROP_DESCRIPTION, "Sets Maximum Distance for check in Metern");
    SetPropertyFloat(PROPERTY_MAX_DISTANCE  NSSUBPROP_MIN, 0);
    SetPropertyFloat(PROPERTY_MAX_DISTANCE  NSSUBPROP_MAX, 10);

    SetPropertyFloat(PROPERTY_MAX_DEPTH_DIFF_BETWEEN_NEIGHBOURS, PROPERTY_DEFAULT_DEPTH_NEIGHBOURS);
    SetPropertyStr(PROPERTY_MAX_DEPTH_DIFF_BETWEEN_NEIGHBOURS  NSSUBPROP_DESCRIPTION,
                   "Maximum Depth Difference in Metern Between Neighbourstixels");
    SetPropertyFloat(PROPERTY_MAX_DEPTH_DIFF_BETWEEN_NEIGHBOURS  NSSUBPROP_MIN, 0);
    SetPropertyFloat(PROPERTY_MAX_DEPTH_DIFF_BETWEEN_NEIGHBOURS  NSSUBPROP_MAX, 1);

    SetPropertyInt(PROPERTY_MIN_STIXELS_FOR_ROI, minStixelsForRoi);
    SetPropertyStr(PROPERTY_MIN_STIXELS_FOR_ROI  NSSUBPROP_DESCRIPTION, "Minimum Stixels for an ROI");
    SetPropertyInt(PROPERTY_MIN_STIXELS_FOR_ROI  NSSUBPROP_MIN, 1);
    SetPropertyInt(PROPERTY_MIN_STIXELS_FOR_ROI  NSSUBPROP_MAX, 1000);
}

tResult StixelsToRoisFilter::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {

        cObjectPtr<IMediaType> pInputType;
        RETURN_IF_FAILED(AllocMediaType(&pInputType, MEDIA_TYPE_CAMERA_PROPERTY, MEDIA_SUBTYPE_CAMERA_PROPERTY,
                                        __exception_ptr));
        // create and register the input pin
        RETURN_IF_FAILED(inputPinCameraProperty.Create("Camera_Property", pInputType, this));
        RETURN_IF_FAILED(RegisterPin(&inputPinCameraProperty));
        pInputType = NULL;

        RETURN_IF_FAILED(AllocMediaType(&pInputType,
                                        MEDIA_TYPE_STIXEL_DATA, MEDIA_SUBTYPE_STIXEL_DATA,
                                        __exception_ptr));
        // create and register the input pin
        RETURN_IF_FAILED(stixelsInputPin.Create("Stixels", pInputType, this));
        RETURN_IF_FAILED(RegisterPin(&stixelsInputPin));

        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid **) &pDescManager, __exception_ptr));

        //get description for position pin
        tChar const *positionString = pDescManager->GetMediaDescription("tPosition");
        RETURN_IF_POINTER_NULL(positionString);

        //get mediatype for position pin
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tPosition", positionString,
                                                                 IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        // set the description for the position pin
        RETURN_IF_FAILED(
                pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &positionDescription));

        // create the input pin
        RETURN_IF_FAILED(positionPin.Create("Position", pTypeSignalValue, static_cast<IPinEventSink *> (this)));
        RETURN_IF_FAILED(RegisterPin(&positionPin));

        RETURN_IF_FAILED(
                rgbInputPin.Create("RGB_in", IPin::PD_Input, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&rgbInputPin));

        // create and register output pins
        cObjectPtr<IMediaType> pOutputType;
        RETURN_IF_FAILED(
                AllocMediaType(&pOutputType, MEDIA_TYPE_OBSTACLE_DATA, MEDIA_SUBTYPE_OBSTACLE_DATA, __exception_ptr));
        RETURN_IF_FAILED(obstacleWorldMapOutputPin.Create("Obstacles_World_Map", pOutputType, this));
        RETURN_IF_FAILED(RegisterPin(&obstacleWorldMapOutputPin));

        RETURN_IF_FAILED(rgbOutputPin.Create("RGB_out", IPin::PD_Output,
                                             static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&rgbOutputPin));

        RETURN_IF_FAILED(rgbDebugOutputPin.Create("Debug_RGB_out", IPin::PD_Output,
                                                  static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&rgbDebugOutputPin));

    } else if (eStage == StageGraphReady) {
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(rgbInputPin.GetMediaType(&pType));
        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid **) &pTypeVideo));
        updateInputImageFormat(pTypeVideo->GetFormat(), rgbInputBitmapFormat, rgbInputImage);
    }
    RETURN_NOERROR;
}

tResult StixelsToRoisFilter::Start(ucom::IException **__exception_ptr) {
    imageUpdated = false;
    stixelsUpdated = false;
    return cFilter::Start(__exception_ptr);
}

tResult StixelsToRoisFilter::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
                                        IMediaSample *pMediaSample) {
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        if (pSource == &inputPinCameraProperty) {
            {
                __sample_read_lock(pMediaSample, tCameraProperties, pData);
                memcpy(&cProp, pData, sizeof(tCameraProperties));
                cPropUpdated = true;
            }
        } else if (cPropUpdated) {
            if (pSource == &positionPin) {
                {
                    __adtf_sample_read_lock_mediadescription(positionDescription, pMediaSample, pCoderInput);
                    if (!positionIDsSet) {
                        pCoderInput->GetID("f32x", m_szIDPositionF32X);
                        pCoderInput->GetID("f32y", m_szIDPositionF32Y);
                        pCoderInput->GetID("f32radius", m_szIDPositionF32Radius);
                        pCoderInput->GetID("f32speed", m_szIDPositionF32Speed);
                        pCoderInput->GetID("f32heading", m_szIDPositionF32Heading);
                        positionIDsSet = tTrue;
                    }
                    pCoderInput->Get(m_szIDPositionF32X, (tVoid *) &pos.point.x);
                    pCoderInput->Get(m_szIDPositionF32Y, (tVoid *) &pos.point.y);
                    pCoderInput->Get(m_szIDPositionF32Radius, (tVoid *) &pos.radius);
                    pCoderInput->Get(m_szIDPositionF32Speed, (tVoid *) &pos.speed);
                    pCoderInput->Get(m_szIDPositionF32Heading, (tVoid *) &pos.heading);
                }
            }
            if (!imageUpdated && pSource == &rgbInputPin) {
                RETURN_IF_FAILED(
                        bufferToMat(rgbInputPin.GetFormat(), pMediaSample, rgbInputImage,
                                    rgbInputBitmapFormat));
                imageUpdated = true;
            } else if (!stixelsUpdated && pSource == &stixelsInputPin) {
                htwk::recieveVector(pMediaSample, tStixelVector);
                stixelsTimestamp = pMediaSample->GetTime();
                stixelsUpdated = true;
            }
            if (stixelsUpdated && imageUpdated) {
                if (!tStixelVector.empty()) {
                    obstacleWorld.clear();
                    extractRois();
                    RETURN_IF_FAILED(transmitVideo(rgbInputImage, rgbOutputPin, rgbOutputBitmapFormat));
                    RETURN_IF_FAILED(transmitObstacles());
                    tStixelVector.clear();
                }
                imageUpdated = false;
                stixelsUpdated = false;
            }
        } else {
            LOG_ERROR("CameraProperties not updated!!");
        }
        if (rgbDebugOutputPin.IsConnected()) {
            debugDrawRoisRGB();
        }
    } else if (nEventCode == IPinEventSink::PE_MediaTypeChanged) {
        if (pSource == &rgbInputPin) {
            RETURN_IF_FAILED(
                    updateInputImageFormat(rgbInputPin.GetFormat(), rgbInputBitmapFormat, rgbInputImage));
        }
    }
    RETURN_NOERROR;
}

void StixelsToRoisFilter::extractRois() {
    if (tStixelVector.size() < 2) {
        return;
    }
    if (!isAlreadyInitalized) {
        initalizeMemberVariables(tStixelVector);
        isAlreadyInitalized = true;
    }

    tUInt64 currentID = dataID;
    for (unsigned int startIndex = ignoreLeftOffset; startIndex < tStixelVector.size(); startIndex++) {
        Point2f topLeft = Point(tStixelVector[startIndex].column, tStixelVector[startIndex].rowTop);
        Point2f bottomRight = Point(tStixelVector[startIndex].column, tStixelVector[startIndex].rowBottom);
        float dispLeft = static_cast<float>(tStixelVector[startIndex].disparity);
        float dispRight = static_cast<float>(tStixelVector[startIndex].disparity);
        float midDisp = dispLeft;
        float maxDisp = dispLeft;
        if (dispLeft == failureDisp || (dispLeft > minDisparity && dispLeft < maxDisparity)) {
            auto checkIndex = startIndex;  //holds last index with correct Disparity
            for (auto stopIndex = startIndex + 1; stopIndex < tStixelVector.size(); stopIndex++) {
                if (checkForBreak(tStixelVector[checkIndex], tStixelVector[stopIndex], topLeft, bottomRight, midDisp)) {
                    break;
                }
                topLeft.y = min(static_cast<int>(topLeft.y), tStixelVector[stopIndex].rowTop); //Update top row
                bottomRight.y = max(static_cast<int>(bottomRight.y), tStixelVector[stopIndex].rowBottom); //Update bottom row
                bottomRight.x = tStixelVector[stopIndex].column; //Update column
                if (dispLeft == failureDisp) { //update left disparity
                    dispLeft = static_cast<float>(tStixelVector[stopIndex].disparity);
                }
                if (tStixelVector[stopIndex].disparity != failureDisp) {
                    //update right disparity to latest possible value
                    dispRight = static_cast<float>(tStixelVector[stopIndex].disparity);
                    midDisp = static_cast<float>((dispLeft + dispRight) / 2.0);
                    maxDisp = max(maxDisp, dispRight);
                }
                //if failureDisp then ROIs should overlap
                if (tStixelVector[stopIndex].disparity != failureDisp || tStixelVector[checkIndex].disparity == failureDisp) {
                    checkIndex = stopIndex;
                }
            }
            if (minStixelsForRoi <= (checkIndex - startIndex) && midDisp > minDisparity && midDisp < maxDisparity) {
                topLeft.x -= stixelWidthOffset;
                bottomRight.x += stixelWidthOffset;
                //check rightBorder
                bottomRight.x = bottomRight.x > upperLimit ? upperLimit : bottomRight.x;

                const Point3f tL3D = getPoint3D(topLeft, dispLeft);
                const Point3f bR3D = getPoint3D(bottomRight, dispRight);
                //check if distance is okay and roi height is okay
                const float height = abs(bR3D.y - tL3D.y);
                if (height < maxHeight && height > minHeight) {
                    tObstacleData obstacle;
                    obstacle.infraredROI.topLeft = topLeft;
                    obstacle.infraredROI.bottomRight = bottomRight;
                    obstacle.rgbROI.topLeft = transferIrPointToRGB(topLeft, dispLeft);
                    obstacle.rgbROI.bottomRight = transferIrPointToRGB(bottomRight, dispRight);

                    Point2f center2D = (topLeft + bottomRight) / 2.0;
                    //needed for radius
                    Point3f center3D = getPoint3D(center2D, maxDisp);
                    //y not needed -> height in Camera, corresponds to z in worldCoords
                    obstacle.obstacle = htwk::translateAndRotate2DPoint(Point2f(center3D.x, center3D.z),
                                                                        static_cast<float>(pos.heading - M_PI_2),
                                                                        infraredToImuTranslation, pos.point);
                    obstacle.height = height;
                    obstacle.radius = abs(bR3D.x - tL3D.x) / 2.0;
                    obstacle.distance = center3D.z;
                    obstacle.id = currentID;
                    obstacleWorld.push_back(obstacle);
                    currentID++;
                }
            }
            startIndex = checkIndex; // ignore already used stixels
        }
    }
    dataID = currentID;
}

const Point3f StixelsToRoisFilter::getPoint3D(const Point2f &pt, const float disp) const {
    const float dist = htwk::disparityToDepth(disp, cProp.inInfraredLeft, cProp.base);
    return htwk::pixelToPoint(pt, dist, cProp.inInfraredLeft);
}

void StixelsToRoisFilter::initalizeMemberVariables(const vector<tStixel> &stixels) {
    initPropertys();
    stixelWidthOffset = ceil((static_cast<float>(stixels[0].column + stixels[1].column)) / 2.0);
    upperLimit = stixels.size() * stixelWidthOffset; //Image Width
    if (stixels.size() % 2 == 0) {
        upperLimit--;
    }
    while (ignoreLeftOffset < stixels.size()) { //Because of Stereo-Matching some left values can't be matched
        if (stixels[ignoreLeftOffset].disparity == failureDisp) {
            ignoreLeftOffset++;
        } else {
            break;
        }
    }
}

void StixelsToRoisFilter::initPropertys() {
    failureDisp = GetPropertyInt(PROPERTY_FAILURE_DISPARITY);
    maxHeight = GetPropertyFloat(PROPERTY_MAX_HEIGHT);
    maxHeightDiff = GetPropertyFloat(PROPERTY_MAX_HEIGHT_DIFF);
    minHeight = GetPropertyFloat(PROPERTY_MIN_HEIGHT);
    minStixelsForRoi = GetPropertyInt(PROPERTY_MIN_STIXELS_FOR_ROI);
    maxDisparityDiffBetweenNeighbours = htwk::depthToDisparity(
            GetPropertyFloat(PROPERTY_MAX_DEPTH_DIFF_BETWEEN_NEIGHBOURS), cProp.inInfraredLeft, cProp.base);
    minDisparity = htwk::depthToDisparity(GetPropertyFloat(PROPERTY_MAX_DISTANCE), cProp.inInfraredLeft, cProp.base);
    maxDisparity = htwk::depthToDisparity(GetPropertyFloat(PROPERTY_MIN_DISTANCE), cProp.inInfraredLeft, cProp.base);
}

tResult StixelsToRoisFilter::transmitObstacles() {
    if (obstacleWorldMapOutputPin.IsConnected()) {
        cObjectPtr<IMediaSample> pOutputObstacleMap;
        if (IS_OK(AllocMediaSample(&pOutputObstacleMap))) {
            pOutputObstacleMap->Update(stixelsTimestamp, &obstacleWorld.front(),
                                       sizeof(tObstacleData) * obstacleWorld.size(), 0);
            RETURN_IF_FAILED(obstacleWorldMapOutputPin.Transmit(pOutputObstacleMap));
        }
    }
    RETURN_NOERROR;
}

tResult StixelsToRoisFilter::transmitVideo(const Mat &image, cVideoPin &pin, tBitmapFormat &bmp) {
    updateOutputImageFormat(image, bmp, pin);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(bmp.nSize));
    RETURN_IF_FAILED(pMediaSample->Update(stixelsTimestamp, image.data, bmp.nSize, IMediaSample::MSF_None));
    RETURN_IF_FAILED(pin.Transmit(pMediaSample));
    RETURN_NOERROR;
}

bool StixelsToRoisFilter::checkForBreak(const tStixel &check, const tStixel &stop, const Point2f &topLeft,
                                        const Point2f &bottomRight, const float &disp) {
    if (check.disparity != failureDisp || stop.disparity != failureDisp) {
        //distance between neighbours not allowed
        if (abs(stop.disparity - check.disparity) >= maxDisparityDiffBetweenNeighbours) {
            return true;
        } //distance not allowed
        else if (disp <= minDisparity && disp >= maxDisparity) {
            return true;
        }
        const float distStop = htwk::disparityToDepth(stop.disparity, cProp.inInfraredLeft, cProp.base);
        const Point3f stopTop3D = htwk::pixelToPoint(Point2f(stop.column, stop.rowTop), distStop, cProp.inInfraredLeft);
        const Point3f stopBottom3D = htwk::pixelToPoint(Point2f(stop.column, stop.rowBottom), distStop,
                                                        cProp.inInfraredLeft);
        // Height is not allowed
        if (stopBottom3D.y - stopTop3D.y >= maxHeight || stopBottom3D.y - stopTop3D.y <= minHeight) {
            return true;
        }
        const float distRoi = htwk::disparityToDepth(disp, cProp.inInfraredLeft, cProp.base);
        const Point3f topLeft3D = htwk::pixelToPoint(topLeft, distRoi, cProp.inInfraredLeft);
        const Point3f bottomRight3D = htwk::pixelToPoint(bottomRight, distRoi, cProp.inInfraredLeft);
        //Stop if stixel height to neighbour stixel is not correct
        if (abs(topLeft3D.y - stopTop3D.y) >= maxHeightDiff ||
            abs(bottomRight3D.y - stopBottom3D.y) >= maxHeightDiff) {
            return true;
        }
    }
    return false;
}

Point2f StixelsToRoisFilter::transferIrPointToRGB(const Point2f &fromPoint, const float &disparity) {
    const float depth = htwk::disparityToDepth(disparity, cProp.inInfraredLeft, cProp.base);
    //2D to 3D
    const Point3f infraredWorld = htwk::pixelToPoint(fromPoint, depth, cProp.inInfraredLeft);
    //3D IR to 3D RGB
    const Point3f rgbWorld = htwk::transformPointToPoint(infraredWorld, cProp.exIRtoRGB);
    //3D to 2D
    const Point2f toPoint = htwk::pointToPixel(rgbWorld, cProp.inRGB);
    return Point2f(checkLimits(LOWER_LIMIT, toPoint.x, upperLimit), checkLimits(LOWER_LIMIT, toPoint.y, UPPER_LIMIT_Y));
}


void StixelsToRoisFilter::debugDrawRoisRGB() {
    if (rgbInputImage.empty()) {
        return;
    }
    Mat roiMat;
    if (rgbInputImage.channels() == 1) { //IR-Cam are CV_8UC1
        cvtColor(rgbInputImage, roiMat, CV_GRAY2BGR);
    } else {
        rgbInputImage.copyTo(roiMat);
    }
    if (!obstacleWorld.empty()) {
        for (const auto &obst : obstacleWorld) {
            //draw rgbROIs
            rectangle(roiMat, obst.rgbROI.topLeft, obst.rgbROI.bottomRight, Scalar(255, 0, 0), 1);
            //draw distance and radius
            ostringstream oss;
            oss << std::setprecision(3) << "D:" << obst.distance << "m R:"
                << obst.radius << "m H:" << obst.height << "m";
            putText(roiMat, oss.str(), Point2f(obst.rgbROI.topLeft.x, obst.rgbROI.bottomRight.y), FONT_HERSHEY_PLAIN, 1,
                    Scalar(0, 0, 200));
            //draw WorldMapCoordinates
            oss.str("");
            oss << std::setprecision(3) << "x:" << obst.obstacle.x << " y:" << obst.obstacle.y;
            putText(roiMat, oss.str(), obst.infraredROI.topLeft, FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 200));
        }
    }
    //Draw position infos
    ostringstream oss;
    oss << std::setprecision(3) << "H:" << (pos.heading * 180) / M_PI << " S:" << pos.speed;
    putText(roiMat, oss.str(), Point2f(20, 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 200));
    oss.str("");
    oss << std::setprecision(3) << "x:" << pos.point.x << " y:" << pos.point.y;
    putText(roiMat, oss.str(), Point2f(20, 40), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 200));
    transmitVideo(roiMat, rgbDebugOutputPin, debugOutputBitmapFormat);
}
