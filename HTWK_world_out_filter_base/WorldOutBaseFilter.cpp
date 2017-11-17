#include "WorldOutBaseFilter.h"

WorldOutBaseFilter::WorldOutBaseFilter(const tChar *info) : cFilter(info) {

}

WorldOutBaseFilter::~WorldOutBaseFilter()
= default;

tResult WorldOutBaseFilter::Init(cFilter::tInitStage eStage, IException **__exception_ptr) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {
        RETURN_IF_FAILED(CreateDescriptions(__exception_ptr));
        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        //RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
    } else if (StageGraphReady == eStage) {
        RETURN_IF_FAILED(
                _runtime->GetObject(OID_WORLD_SERVICE, IID_WORLD_INTERFACE, (tVoid **) &worldService, __exception_ptr));
    }

    RETURN_NOERROR;
}

tResult WorldOutBaseFilter::CreateDescriptions(IException **__exception_ptr) {
    cObjectPtr<IMediaDescriptionManager> descManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                         (tVoid **) &descManager, __exception_ptr));

    //get SignalValue
    tChar const *signalValueDescription = descManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(signalValueDescription);
    typeSignal = new cMediaType(0, 0, 0, "tSignalValue", signalValueDescription,
                                IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(typeSignal->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &descriptionSignal));

    RETURN_NOERROR;
}

tResult WorldOutBaseFilter::CreateInputPins(IException **__exception_ptr) {
    RETURN_IF_FAILED(triggerInput.Create("Trigger_Input", typeSignal, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&triggerInput));

    RETURN_NOERROR;
}

tResult WorldOutBaseFilter::Shutdown(tInitStage eStage, IException **__exception_ptr) {
    if (eStage == StageGraphReady) {
        worldService = nullptr;
    }
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult
WorldOutBaseFilter::OnPinEvent(IPin *sourcePin, tInt eventCode, tInt param1, tInt param2, IMediaSample *mediaSample) {
    if (eventCode != IPinEventSink::PE_MediaSampleReceived) {
        RETURN_NOERROR;
    }

    RETURN_IF_POINTER_NULL(mediaSample);

    if (sourcePin == &triggerInput) {
        tFloat32 intervalTime;

        {
            __adtf_sample_read_lock_mediadescription(descriptionSignal, mediaSample, inputCoder);
            inputCoder->Get("f32Value", (tVoid *) &intervalTime);
        }

        if (IS_OK(OnTrigger(intervalTime))) {
            RETURN_NOERROR;
        }
    }

    RETURN_NOERROR;
}


