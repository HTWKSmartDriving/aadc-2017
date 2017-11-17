#ifndef _HTWK_EMERGENCY_STOP_FILTER_H_
#define _HTWK_EMERGENCY_STOP_FILTER_H_

#include "stdafx.h"
#include <aadc_structs.h>
#include "..//HTWK_Debug/EnableLogs.h"

#define OID_ADTF_TEMPLATE_FILTER "htwk.emergency_filter"
#define FILTER_NAME "HTWK Emergency Stop"

//property defines
#define FLEXIBLE_BRAKING_PROPERTY "flexible braking"
#define FLEXIBLE_BRAKING_DEFAULT true

#define ULTRASONIC_PROPERTY "fixed braking distance"
#define ULTRASONIC_DEFAULT 30
#define ULTRASONIC_DESCRIPTION "ignored if flexible value is true"
#define ULTRASONIC_MIN 10
#define ULTRASONIC_MAX 400

#define MIN_US_DISTANCE_PROPERTY "min distance of ultrasonic sensors"
#define MIN_US_DISTANCE_DEFAULT 6
#define MIN_US_DISTANCE_DESCRIPTION "distance of ultrasonic, at which the car will drive independet from obstacle recognition"
#define MIN_US_DISTANCE_MIN 3
#define MIN_US_DISTANCE_MAX 10

#define MIN_BRAKE_DISTANCE_PROPERTY "min distance at which the car starts braking"
#define MIN_BRAKE_DISTANCE_DEFAULT 9
#define MIN_BRAKE_DISTANCE_DESCRIPTION "only used if flexBrake is enabled, changes stopping distance at each speed"
#define MIN_BRAKE_DISTANCE_MIN 9
#define MIN_BRAKE_DISTANCE_MAX 20


//defines for braking formula
#define BRAKE_SLOPE_SQUARE 29
#define BRAKE_SLOPE_LINEAR 8



class EmergencyStop : public adtf::cFilter {
ADTF_FILTER(OID_ADTF_TEMPLATE_FILTER, "HTWK Emergency Stop", adtf::OBJCAT_Auxiliary);

public:
    EmergencyStop(const tChar *__info);

    virtual ~EmergencyStop();

protected:
    tResult Init(tInitStage eStage, ucom::IException **__exception_ptr);
    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

private:
    cInputPin inputWantedSpeed;
    cInputPin inputDrivenSpeed;
    cInputPin inputUltrasonicStruct;
    cInputPin inputStopFlag;
    cOutputPin outputSpeed;

    tResult CreateDescriptions(IException **__exception_ptr);

    cObjectPtr<IMediaType> wantedSpeed;
    cObjectPtr<IMediaType> drivenSpeed;
    cObjectPtr<IMediaType> stopFlag;
    cObjectPtr<IMediaType> ultrasonicStruct;

    tSignalValue receivedWantedSpeed;
    tSignalValue receivedDrivenSpeed;
    tBoolSignalValue receivedStopFlag;
    tUltrasonicStruct receivedUltrasonicStruct;

    //property values
    tBool flexBrakePropValue;
    int minFixedUsPropValue;
    tFloat64 donotBrakeDistancePropValue;
    int minBrakeDistancePropValue;



    bool checkFront(const tUltrasonicStruct &ultrasonicStruct, const tFloat64 &checkValue);
    bool FrontFree(const tUltrasonicStruct &ultrasonicStruct, const tFloat32 &brakingDistance);

    bool checkBack(const tUltrasonicStruct &ultrasonicStruct, const tFloat64 &checkValue);
    bool BackFree(const tUltrasonicStruct &ultrasonicStruct, const tFloat32 &brakingDistance);
    tFloat32 BrakingDistance(const tFloat32 &speed);
    bool DrivePossible(const tBoolSignalValue &stopFlag,const tSignalValue &drivenSpeed, const tUltrasonicStruct &ultrasonicStruct,
                       const tSignalValue &wantedSpeed);


    };

#endif // _HTWK_EMERGENCY_STOP_FILTER_H_
