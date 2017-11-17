#include "EmergenyStop.h"

/// Create filter shell
ADTF_FILTER_PLUGIN(FILTER_NAME, OID_ADTF_TEMPLATE_FILTER, EmergencyStop);


EmergencyStop::EmergencyStop(const tChar *__info) : cFilter(__info) {

    //Property for enabling/disabling usage of flexible braking distance
    SetPropertyBool(FLEXIBLE_BRAKING_PROPERTY, FLEXIBLE_BRAKING_DEFAULT);

    //Property for fixed braking distance, only used if other property == false
    SetPropertyInt(ULTRASONIC_PROPERTY, ULTRASONIC_DEFAULT);
    SetPropertyStr(ULTRASONIC_PROPERTY NSSUBPROP_DESCRIPTION, ULTRASONIC_DESCRIPTION);
    SetPropertyInt(ULTRASONIC_PROPERTY  NSSUBPROP_MIN, ULTRASONIC_MIN);
    SetPropertyInt(ULTRASONIC_PROPERTY  NSSUBPROP_MAX, ULTRASONIC_MAX);

    //Property for distance, in which obstacles get ignored (correcting false positives of ultrasonic sensors)
    SetPropertyFloat(MIN_US_DISTANCE_PROPERTY, MIN_US_DISTANCE_DEFAULT);
    SetPropertyStr(MIN_US_DISTANCE_PROPERTY NSSUBPROP_DESCRIPTION, MIN_US_DISTANCE_DESCRIPTION);
    SetPropertyFloat(MIN_US_DISTANCE_PROPERTY  NSSUBPROP_MIN, MIN_US_DISTANCE_MIN);
    SetPropertyFloat(MIN_US_DISTANCE_PROPERTY  NSSUBPROP_MAX, MIN_US_DISTANCE_MAX);

    //Property for minimum distance, at which car starts braking
    SetPropertyFloat(MIN_BRAKE_DISTANCE_PROPERTY, MIN_BRAKE_DISTANCE_DEFAULT);
    SetPropertyStr(MIN_BRAKE_DISTANCE_PROPERTY NSSUBPROP_DESCRIPTION, MIN_BRAKE_DISTANCE_DESCRIPTION);
    SetPropertyFloat(MIN_BRAKE_DISTANCE_PROPERTY  NSSUBPROP_MIN, MIN_BRAKE_DISTANCE_MIN);
    SetPropertyFloat(MIN_BRAKE_DISTANCE_PROPERTY  NSSUBPROP_MAX, MIN_BRAKE_DISTANCE_MAX);
}

EmergencyStop::~EmergencyStop() {

}

tResult EmergencyStop::Init(tInitStage eStage, __exception) {
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst) {
        RETURN_IF_FAILED(CreateDescriptions(__exception_ptr));

        // create and register the speed input pin
        RETURN_IF_FAILED(inputWantedSpeed.Create("Speed", wantedSpeed, this));
        RETURN_IF_FAILED(RegisterPin(&inputWantedSpeed));

        // create and register the stopFlag input pin
        RETURN_IF_FAILED(inputStopFlag.Create("StopFlag", stopFlag, this));
        RETURN_IF_FAILED(RegisterPin(&inputStopFlag));

        // create and register the ultrasonic input pin
        RETURN_IF_FAILED(inputUltrasonicStruct.Create("UltrasonicStruct", ultrasonicStruct, this));
        RETURN_IF_FAILED(RegisterPin(&inputUltrasonicStruct));

        // create and register the driven speed input pin
        RETURN_IF_FAILED(inputDrivenSpeed.Create("driven_Speed", drivenSpeed, this));
        RETURN_IF_FAILED(RegisterPin(&inputDrivenSpeed));

        // create and register the output pin
        RETURN_IF_FAILED(outputSpeed.Create("SpeedOutput", wantedSpeed, this));
        RETURN_IF_FAILED(RegisterPin(&outputSpeed));

        receivedStopFlag.bValue = false;
        flexBrakePropValue = GetPropertyBool(FLEXIBLE_BRAKING_PROPERTY);
        minFixedUsPropValue = GetPropertyInt(ULTRASONIC_PROPERTY);
        donotBrakeDistancePropValue = GetPropertyFloat(MIN_US_DISTANCE_PROPERTY);
        minBrakeDistancePropValue = GetPropertyInt(MIN_BRAKE_DISTANCE_PROPERTY);


    }

    RETURN_NOERROR;
}


tResult
EmergencyStop::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) {
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

        // by comparing it to our member pin variable we can find out which pin received
        // the sample
        if (pSource == &inputWantedSpeed) {

            {
                // this will aquire the read lock on the sample and declare and initialize a pointer to the data
                __sample_read_lock(pMediaSample, tSignalValue, pData);
                // now we can access the sample data through the pointer
                receivedWantedSpeed = *pData;
            }

            // check if driving is possible
            if (!EmergencyStop::DrivePossible(receivedStopFlag, receivedDrivenSpeed, receivedUltrasonicStruct,
                                              receivedWantedSpeed)) {
                // set speed to 0
                receivedWantedSpeed.f32Value = 0;
            }
            //transmit speed to ouput
            cObjectPtr<IMediaSample> outputSpeedValue;
            if (IS_OK(AllocMediaSample(&outputSpeedValue))) {
                // now set its data
                // we reuse the timestamp from the incoming media sample. Please see the api documentation
                // (ADTF Extreme Programmers -> The ADTF Streamtime) for further reference on how sample times are handled in ADTF
                outputSpeedValue->Update(pMediaSample->GetTime(), &receivedWantedSpeed, sizeof(tSignalValue), 0);

                // and now we can transmit it
                outputSpeed.Transmit(outputSpeedValue);
            }

        } else if (pSource == &inputDrivenSpeed) {

            {// this will aquire the read lock on the sample and declare and initialize a pointer to the data
                __sample_read_lock(pMediaSample, tSignalValue, pData);
                // now we can access the sample data through the pointer
                receivedDrivenSpeed = *pData;
            }
        } else if (pSource == &inputStopFlag) {

            {// this will aquire the read lock on the sample and declare and initialize a pointer to the data
                __sample_read_lock(pMediaSample, tBoolSignalValue, pData);
                // now we can access the sample data through the pointer
                receivedStopFlag = *pData;
            }
        } else if (pSource == &inputUltrasonicStruct) {
            {// this will aquire the read lock on the sample and declare and initialize a pointer to the data
                __sample_read_lock(pMediaSample, tUltrasonicStruct, pData);
                // now we can access the sample data through the pointer
                receivedUltrasonicStruct = *pData;
            }
        }
    }
    RETURN_NOERROR;
}

tResult EmergencyStop::CreateDescriptions(IException **__exception_ptr) {
    cObjectPtr<IMediaDescriptionManager> descManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                         (tVoid **) &descManager, __exception_ptr));

    //get Speed
    tChar const *speedDescription = descManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(speedDescription);
    wantedSpeed = new cMediaType(0, 0, 0, "tSignalValue", speedDescription, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(wantedSpeed->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &speedDescription));

    //get stopFlag
    tChar const *stopFlagDescription = descManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(stopFlagDescription);
    stopFlag = new cMediaType(0, 0, 0, "tBoolSignalValue", stopFlagDescription,
                              IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(stopFlag->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &stopFlagDescription));

    // get ultrasonicStruct
    tChar const *ultrasonicStructDescription = descManager->GetMediaDescription("tUltrasonicStruct");
    RETURN_IF_POINTER_NULL(ultrasonicStructDescription);
    ultrasonicStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", ultrasonicStructDescription,
                                      IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(
            ultrasonicStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &ultrasonicStructDescription));

    //get Speed
    tChar const *drivenSpeedDescription = descManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(drivenSpeedDescription);
    drivenSpeed = new cMediaType(0, 0, 0, "tSignalValue", drivenSpeedDescription,
                                 IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(drivenSpeed->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &drivenSpeedDescription));


    RETURN_NOERROR;
}


tFloat32 EmergencyStop::BrakingDistance(const tFloat32 &speed) {
    if (flexBrakePropValue) {
        // braking formula is empirical tested and developed
        // made some tests with the braking distance of the model car
        // calculated trend formula from the values
        // formula returns value, which is greater than the real braking distance to avoid collisions
        return BRAKE_SLOPE_SQUARE * (speed * speed) + BRAKE_SLOPE_LINEAR * std::abs(speed) + minBrakeDistancePropValue;

    } else {
        return minFixedUsPropValue;
    }
}


/// check if no obstacle is in front
/// return true if way is clear
bool EmergencyStop::checkFront(const tUltrasonicStruct &ultrasonicStruct, const tFloat64 &checkValue) {
    return ultrasonicStruct.tFrontCenterRight.f32Value > checkValue &&
           ultrasonicStruct.tFrontCenter.f32Value > checkValue &&
           ultrasonicStruct.tFrontCenterLeft.f32Value > checkValue;
};


bool EmergencyStop::FrontFree(const tUltrasonicStruct &ultrasonicStruct, const tFloat32 &brakingDistance) {
    if (checkFront(ultrasonicStruct,donotBrakeDistancePropValue)) {
        return checkFront(ultrasonicStruct,brakingDistance);
    } else {
        // return true, if ultrasonic sensor values < MIN_US_DISTANCE_PROPERTY
        // removes false positives
        // should not make trouble because if car gets closer as 9 cm it starts braking,
        // car should never come this close and ultrasonic sensor values may be wrong at this distance
        return true;
    }
}

///     check if no obstacle is in back
///     return true if way is clear
bool EmergencyStop::checkBack(const tUltrasonicStruct &ultrasonicStruct, const tFloat64 &checkValue) {
    return ultrasonicStruct.tRearRight.f32Value > checkValue &&
           ultrasonicStruct.tRearCenter.f32Value > checkValue &&
           ultrasonicStruct.tRearLeft.f32Value > checkValue;
};


bool EmergencyStop::BackFree(const tUltrasonicStruct &ultrasonicStruct, const tFloat32 &brakingDistance) {


    if (checkBack(ultrasonicStruct,donotBrakeDistancePropValue)) {
        return checkBack(ultrasonicStruct,brakingDistance);
    } else {
        // same as at EmergencyStop::FrontFree()
        return true;
    }
}


bool
EmergencyStop::DrivePossible(const tBoolSignalValue &stopFlag, const tSignalValue &drivenSpeed,
                             const tUltrasonicStruct &ultrasonicStruct,
                             const tSignalValue &wantedSpeed) {
    if (stopFlag.bValue) {
        return false;
    } else {
        tFloat32 brakingDistance = EmergencyStop::BrakingDistance(drivenSpeed.f32Value);
//        check if driving forward
        if (drivenSpeed.f32Value > 0.01) {
            return FrontFree(ultrasonicStruct, brakingDistance);
//        check if driving backward
        } else if (drivenSpeed.f32Value < -0.01) { //drive backward
            return BackFree(ultrasonicStruct, brakingDistance);
//        check if standing
        } else {
            brakingDistance = EmergencyStop::BrakingDistance(wantedSpeed.f32Value);
//          check if want to drive forward
            if (wantedSpeed.f32Value > 0.01) {
                return FrontFree(ultrasonicStruct, brakingDistance);
//                or want to drive backward
            } else if (wantedSpeed.f32Value < 0.01) {
                return BackFree(ultrasonicStruct, brakingDistance);
            } else {
//                if car is standing and it should remain standing
                return false;
            }
        }
    }
}
