#include "CropObjects.h"
#include "../../../HTWK_Utils/ImageHelper.h"

ADTF_FILTER_PLUGIN(OID_ADTF_FILTER_DEF, OID_ADTF_FILTER_DEF, CropObjects);

CropObjects::CropObjects(const tChar* __info) : cFilter(__info), useHull(false) {
    imageIdx = 0;
    SetPropertyStr("FilePath", "Example: /user/Desktop/Videos"); //Point to an existing directory!
    SetPropertyBool("FilePath" NSSUBPROP_DIRECTORY, tTrue);
    SetPropertyStr("FilePath" NSSUBPROP_DESCRIPTION, "Directory in which the video should be saved.");
    SetPropertyStr("FileName", "Prefix");
    SetPropertyStr("FileName" NSSUBPROP_DESCRIPTION, "The filename which will be appended");
    SetPropertyBool("Use convex hull", tFalse);
}

CropObjects::~CropObjects() {}

void CropObjects::Crop() {
    for (const auto& obst : obstacleData) {
        const cv::Rect roi(obst.rgbROI.topLeft, obst.rgbROI.bottomRight);
        if (!(   0 <= roi.x && 0 < roi.width && roi.x + roi.width <= inputImage.cols
              && 0 <= roi.y && 0 < roi.height && roi.y + roi.height <= inputImage.rows))
            continue;
        cv::Mat cropped;
        if (useHull) {
            cv::Mat mask(roi.height, roi.width, CV_8UC1, cv::Scalar(0));
            cv::fillConvexPoly(mask, &obst.hull[0], static_cast<int>(obst.hullSize), cv::Scalar(255));
            cropped = cv::Mat(roi.height, roi.width, CV_8UC3, cv::Scalar(104, 117, 124));
            inputImage(roi).copyTo(cropped, mask);
        } else {
            cropped = inputImage(roi).clone();
        }
        const std::string finalPath = strTargetPath + std::to_string(imageIdx) + ".ppm";
        cv::imwrite(finalPath, cropped);
        imageIdx++;
    }
}

tResult CropObjects::OnPinEvent(IPin* pSource, tInt nEventCode, tInt /*nParam1*/, tInt /*nParam2*/,
                                IMediaSample* pMediaSample) {
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    bool newData = false;
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        if (pSource == &imageInputPin) {
            RETURN_IF_FAILED(bufferToMat(imageInputPin.GetFormat(), pMediaSample,
                                         inputImage, imageInputFormat));
            timeImage = pMediaSample->GetTime();
            newData = true;
        } else if (pSource == &obstacleInputPin) {
            __adtf_sample_read_lock(pMediaSample, tObstacleData, pData);
            const int roiCount = pMediaSample->GetSize() / sizeof(tObstacleData);
            obstacleData.clear();
            obstacleData.reserve(roiCount);
            for (int i = 0; roiCount > i; ++i) {
                obstacleData.push_back(pData[i]);
            }
            timeObstacle = pMediaSample->GetTime();
            newData = true;
        }
        if (newData && timeObstacle == timeImage) {
            Crop();
        }
    } else if (nEventCode == IPinEventSink::PE_MediaTypeChanged) {
        if (pSource == &imageInputPin) {
            RETURN_IF_FAILED(updateInputImageFormat(imageInputPin.GetFormat(), imageInputFormat,
                                                    inputImage));
        }
    }

    RETURN_NOERROR;
}

tResult CropObjects::Init(cFilter::tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {
        {
            cObjectPtr<IMediaType> pInputType;
            RETURN_IF_FAILED(AllocMediaType(&pInputType, MEDIA_TYPE_OBSTACLE_DATA, MEDIA_SUBTYPE_OBSTACLE_DATA,
                                            __exception_ptr));
            RETURN_IF_FAILED(obstacleInputPin.Create("Obstacles", pInputType, this));
            RETURN_IF_FAILED(RegisterPin(&obstacleInputPin));
        }
        {
            // Video Input
            RETURN_IF_FAILED(
                    imageInputPin.Create("Image", IPin::PD_Input, static_cast<IPinEventSink *>(this)));
            RETURN_IF_FAILED(RegisterPin(&imageInputPin));
        }

    } else if (eStage == StageGraphReady) {
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(imageInputPin.GetMediaType(&pType));
        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid **) &pTypeVideo));
        updateInputImageFormat(pTypeVideo->GetFormat(), imageInputFormat, inputImage);
    }

    RETURN_NOERROR;
}

tResult CropObjects::Shutdown(cFilter::tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));
    RETURN_NOERROR;
}

tResult CropObjects::Start(__exception) {
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));
    cFilename absoluteFilePath = GetPropertyStr("FilePath");
    ADTF_GET_CONFIG_FILENAME(absoluteFilePath);
    absoluteFilePath.AppendTrailingSlash();
    strTargetPath = std::string(absoluteFilePath) + GetPropertyStr("FileName");
    useHull = GetPropertyBool("Use convex hull");
    RETURN_NOERROR;
}

tResult CropObjects::Stop(__exception) {
    RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));
    RETURN_NOERROR;
}
