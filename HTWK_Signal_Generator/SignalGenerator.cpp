#include "SignalGenerator.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, SignalGenerator)

SignalGenerator::SignalGenerator(const tChar *__info) : cTimeTriggeredFilter(__info)
{
    SetPropertyInt(INTERVAL_PROPERTY, INTERVAL_DEFAULT);
    SetPropertyStr(INTERVAL_PROPERTY NSSUBPROP_DESCRIPTION, "Interval time in ms");
    SetPropertyInt(INTERVAL_PROPERTY NSSUBPROP_MIN, 10);

    SetPropertyFloat(SIGNAL_PROPERTY, SIGNAL_DEFAULT);
    SetPropertyBool(SIGNAL_PROPERTY NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(SIGNAL_PROPERTY NSSUBPROP_DESCRIPTION, "Signal to transmit");
}

SignalGenerator::~SignalGenerator()
= default;

tResult SignalGenerator::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (StageFirst == eStage)
    {
        RETURN_IF_FAILED(CreateDescriptions(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
    }
    else if (StageNormal == eStage)
    {
        auto interval = tUInt(GetPropertyInt(INTERVAL_PROPERTY));
        SetInterval(interval * 1000);
    }
    else if (StageGraphReady == eStage)
    {

    }

    RETURN_NOERROR;
}

tResult SignalGenerator::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage, __exception_ptr);
}


tResult SignalGenerator::CreateDescriptions(IException **__exception_ptr)
{
    cObjectPtr<IMediaDescriptionManager> descManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                         (tVoid **) &descManager, __exception_ptr));

    //get SignalValue
    tChar const *signalValueDescription = descManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(signalValueDescription);
    signalValueStruct = new cMediaType(0, 0, 0, "tSignalValue", signalValueDescription,
                                   IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(signalValueStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &signalDescription));

    RETURN_NOERROR;
}

tResult SignalGenerator::CreateOutputPins(IException **__exception_ptr) {
    RETURN_IF_FAILED(signalPin.Create("signal", signalValueStruct, static_cast<IPinEventSink *> (this)));
    RETURN_IF_FAILED(RegisterPin(&signalPin));

    RETURN_NOERROR;
}

tResult SignalGenerator::Cycle(IException **__exception_ptr) {

    const auto value = static_cast<const tFloat32>(GetPropertyFloat(SIGNAL_PROPERTY));
    TransmitValue(signalPin, value);

    RETURN_NOERROR;
}

tResult SignalGenerator::TransmitValue(cOutputPin &pin, tFloat32 value)
{
    if (!pin.IsConnected())
    {
        RETURN_NOERROR;
    }

    cObjectPtr<IMediaSample> mediaSample;
    AllocMediaSample((tVoid **) &mediaSample);

    cObjectPtr<IMediaSerializer> serializer;
    signalDescription->GetMediaSampleSerializer(&serializer);
    mediaSample->AllocBuffer(serializer->GetDeserializedSize());

    tFloat32 f32Value = value;
    tUInt32 ui32TimeStamp = 0;

    {
        __adtf_sample_write_lock_mediadescription(signalDescription, mediaSample, pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid *) &f32Value);
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid *) &(ui32TimeStamp));
    }

    mediaSample->SetTime(_clock->GetStreamTime());
    pin.Transmit(mediaSample);

    RETURN_NOERROR;
}