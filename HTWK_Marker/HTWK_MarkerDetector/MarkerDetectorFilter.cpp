#include "MarkerDetectorFilter.h"

ADTF_FILTER_PLUGIN(OID_ADTF_FILTER_DEF, OID_ADTF_FILTER_DEF, MarkerDetectorFilter)

MarkerDetectorFilter::MarkerDetectorFilter(const tChar *__info) : cFilter(__info) {
    SetPropertyInt(PROPERTY_DELAY, PROPERTY_DEFAULT_DELAY);
    SetPropertyStr(PROPERTY_DELAY NSSUBPROP_DESCRIPTION, "Delay in ms");
    SetPropertyInt(PROPERTY_DELAY NSSUBPROP_MIN, 0);
    SetPropertyInt(PROPERTY_DELAY NSSUBPROP_MAX, 1000);

    SetPropertyStr("Calibration File for used Camera", "");
    SetPropertyBool("Calibration File for used Camera" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Calibration File for used Camera" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER,
                   "YML Files (*.yml)");
    SetPropertyStr("Calibration File for used Camera" NSSUBPROP_DESCRIPTION,
                   "Here you have to set the file with calibration paraemters of the used camera");

    SetPropertyStr("Detector Paramater File", "");
    SetPropertyBool("Detector Paramater File" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Detector Paramater File" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "YML Files (*.yml)");
    SetPropertyStr("Detector Paramater File" NSSUBPROP_DESCRIPTION,
                   "Here you have to set the file with the parameters with the detector params");

    SetPropertyFloat("Size of Markers", 0.117f);
    SetPropertyStr("Size of Markers" NSSUBPROP_DESCRIPTION, "Size (length of one side) of markers in m");

    SetPropertyBool("Crop image", tFalse);
    SetPropertyStr("Crop image" NSSUBPROP_DESCRIPTION,
                   "Crop image to relevant area before searching for markers (better performance, possibly small accuracy loss)");
}

MarkerDetectorFilter::~MarkerDetectorFilter() {
}

tResult MarkerDetectorFilter::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
    if (eStage == StageFirst) {
        RETURN_IF_FAILED(
                inputVideoPin.Create("Undistorted_Video_Input", IPin::PD_Input, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&inputVideoPin));

        RETURN_IF_FAILED(
                outputDebugVideoPin.Create("Video_Debug_Output", IPin::PD_Output, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&outputDebugVideoPin));

        // create the description manager
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid **) &pDescManager, __exception_ptr));
        // create the description for the road sign pin
        tChar const *strDescExt = pDescManager->GetMediaDescription("tRoadSignExt");
        RETURN_IF_POINTER_NULL(strDescExt);
        cObjectPtr<IMediaType> pTypeExt = new cMediaType(0, 0, 0, "tRoadSignExt", strDescExt,
                                                         IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        // create the extended road sign OutputPin
        RETURN_IF_FAILED(m_oPinRoadSignExt.Create("RoadSign_ext", pTypeExt, this));
        RETURN_IF_FAILED(RegisterPin(&m_oPinRoadSignExt));
        // set the description for the extended road sign pin
        RETURN_IF_FAILED(
                pTypeExt->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &m_pDescriptionRoadSignExt));

    } else if (eStage == StageNormal) {
        markerSize = static_cast<tFloat32>(GetPropertyFloat("Size of Markers"));

        cFilename fileDetectorParameter = GetPropertyStr("Detector Paramater File");
        if (fileDetectorParameter.IsEmpty()) {
            THROW_ERROR_DESC(ERR_INVALID_FILE, "Detector Parameter File for Markers not set");
        }
        ADTF_GET_CONFIG_FILENAME(fileDetectorParameter);
        fileDetectorParameter = fileDetectorParameter.CreateAbsolutePath(".");
        if (!(cFileSystem::Exists(fileDetectorParameter))) {
            THROW_ERROR_DESC(ERR_INVALID_FILE, "Detector Parameter file for Markers not found");
        }
        detectorParams = aruco::DetectorParameters::create();
        if (!(readDetectorParameters(fileDetectorParameter.GetPtr(), detectorParams))) {
            THROW_ERROR_DESC(ERR_INVALID_FILE, "Detector Parameter file not valid");
        }
        dictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);

        cFilename fileCalibration = GetPropertyStr("Calibration File for used Camera");

        if (fileCalibration.IsEmpty()) {
            THROW_ERROR_DESC(ERR_INVALID_FILE, "Calibration File for camera not set");
        }

        ADTF_GET_CONFIG_FILENAME(fileCalibration);
        fileCalibration = fileCalibration.CreateAbsolutePath(".");
        if (!(cFileSystem::Exists(fileCalibration))) {
            THROW_ERROR_DESC(ERR_INVALID_FILE, "Calibration File for camera not found");
        }

        if (!readCameraParameters(fileCalibration.GetPtr(), intrinsic, distortion)) {
            THROW_ERROR_DESC(ERR_INVALID_FILE, "Calibration File couldnt be loaded");
        }

    } else if (eStage == StageGraphReady) {
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(inputVideoPin.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid **) &pTypeVideo));

        updateInputImageFormat(pTypeVideo->GetFormat(), inputBitmapFormat, inputImageUnscaled);
        RETURN_IF_FAILED(thread.Create(NULL, 0, cThread::TS_Suspended, static_cast<IThreadFunc *>(this)));
    }
    RETURN_NOERROR;
}

tResult MarkerDetectorFilter::Start(ucom::IException **__exception_ptr) {
    minTimeDifference = static_cast<tTimeStamp>(GetPropertyInt(PROPERTY_DELAY) * 1e3);
    previousTime = 0;
    useCrop = GetPropertyBool("Crop image");
    // IDs were not set yet
    m_bIDsRoadSignExtSet = tFalse;
    return cFilter::Start(__exception_ptr);
}

tResult MarkerDetectorFilter::Stop(ucom::IException **__exception_ptr) {
    if (thread.GetState() == cThread::TS_Running) {
        thread.Suspend(tTrue);
    }
    previousTime = 0;
    return cFilter::Stop(__exception_ptr);
}

tResult MarkerDetectorFilter::Shutdown(tInitStage eStage, __exception) {
    if (eStage == StageNormal) {
        thread.Terminate(tTrue);
        thread.Release();
    }
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult MarkerDetectorFilter::OnPinEvent(IPin *pSource, tInt nEventCode, tInt /*nParam1*/, tInt /*nParam2*/,
                                         IMediaSample *pMediaSample) {
    RETURN_IF_POINTER_NULL(pMediaSample);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        if (pSource == &inputVideoPin) {
            if (pMediaSample->GetTime() - previousTime > minTimeDifference &&
                thread.GetState() == cThread::TS_Suspended) {
                previousTime = pMediaSample->GetTime();
                bufferToMat(inputVideoPin.GetFormat(), pMediaSample, inputImageUnscaled, inputBitmapFormat);
                if (useCrop) {
                    const cv::Rect crop(0, inputImageUnscaled.rows / 2, inputImageUnscaled.cols,
                                        inputImageUnscaled.rows / 4);
                    inputImage = inputImageUnscaled(crop);
                } else {
                    inputImage = inputImageUnscaled;
                }
                RETURN_IF_FAILED(thread.Run());
            }
        }
    } else if (nEventCode == IPinEventSink::PE_MediaTypeChanged) {
        if (pSource == &inputVideoPin) {
            updateInputImageFormat(inputVideoPin.GetFormat(), inputBitmapFormat, inputImageUnscaled);
        }
    }
    RETURN_NOERROR;
}


tResult MarkerDetectorFilter::ThreadFunc(cThread */*Thread*/, tVoid */*data*/, tSize /*size*/) {
    vector<int> ids;
    vector<vector<Point2f> > corners;
    vector<Vec3d> rvecs, tvecs;

    aruco::detectMarkers(inputImage, dictionary, corners, ids, detectorParams);
    if (ids.size() > 0) {
        aruco::estimatePoseSingleMarkers(corners, markerSize, intrinsic, distortion, rvecs, tvecs);
        for (unsigned int i = 0; i < ids.size(); i++) {
            sendRoadSignStructExt(static_cast<tInt16>(ids[i]), getMarkerArea(corners[i]), _clock->GetStreamTime(),
                                  tvecs[i], rvecs[i]);
        }
    }

    if (outputDebugVideoPin.IsConnected()) {
        Mat outputImage;
        outputImage = inputImage.clone();
        updateOutputImageFormat(outputImage, outputBitmapFormat, outputDebugVideoPin);
        aruco::drawDetectedMarkers(outputImage, corners, ids);
        if (ids.size() > 0) {
            for (unsigned int i = 0; i < ids.size(); i++) {
                aruco::drawAxis(outputImage, intrinsic, distortion, rvecs[i], tvecs[i], markerSize * 0.5f);
            }
        }
        cObjectPtr<IMediaSample> pNewSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pNewSample));
        pNewSample->Update(_clock->GetTime(), outputImage.data, outputBitmapFormat.nSize, 0);
        outputDebugVideoPin.Transmit(pNewSample);
    }
    RETURN_IF_FAILED(thread.Suspend());
    RETURN_NOERROR;
}

tResult MarkerDetectorFilter::sendRoadSignStructExt(const tInt16 &i16ID, const tFloat32 &f32MarkerSize,
                                                    const tTimeStamp &timeOfFrame, const Vec3d &Tvec,
                                                    const Vec3d &Rvec) {
    // create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));

    // get the serializer
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionRoadSignExt->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    // alloc the buffer memory
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    {
        // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pDescriptionRoadSignExt, pMediaSample, pCoder);

        // get IDs
        if (!m_bIDsRoadSignExtSet) {
            pCoder->GetID("i16Identifier", m_szIDRoadSignExtI16Identifier);
            pCoder->GetID("f32Imagesize", m_szIDRoadSignExtF32Imagesize);
            pCoder->GetID("af32RVec[0]", m_szIDRoadSignExtAf32RVec);
            pCoder->GetID("af32TVec[0]", m_szIDRoadSignExtAf32TVec);
            m_bIDsRoadSignExtSet = tTrue;
        }

        pCoder->Set(m_szIDRoadSignExtI16Identifier, (tVoid *) &i16ID);
        pCoder->Set(m_szIDRoadSignExtF32Imagesize, (tVoid *) &f32MarkerSize);
        //convert from cv::Vec3D to array
        tFloat32 rvecFl32array[3] = {tFloat32(Rvec[0]), tFloat32(Rvec[1]), tFloat32(Rvec[2])};
        tFloat32 tvecFl32array[3] = {tFloat32(Tvec[0]), tFloat32(Tvec[1]), tFloat32(Tvec[2])};
        pCoder->Set("af32TVec", (tVoid *) &tvecFl32array[0]);
        pCoder->Set("af32RVec", (tVoid *) &rvecFl32array[0]);

        pMediaSample->SetTime(timeOfFrame);
    }
    //doing the transmit
    RETURN_IF_FAILED(m_oPinRoadSignExt.Transmit(pMediaSample));
    RETURN_NOERROR;
}
