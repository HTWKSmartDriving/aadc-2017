//
// Created by lina on 02.10.17.
//

#ifndef AADC_USER_GRIDMAPS_H
#define AADC_USER_GRIDMAPS_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <aadc_structs.h>

using namespace adtf;

#include <adtf_graphics.h>

using namespace adtf_graphics;

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <HTWKPoint.hpp>
#include <ImageHelper.h>
#include "../HTWK_Types/TrackingData.h"
#include <VectorHelper.h>

#define OID_ADTF_FILTER_DEF "htwk.occ_grid_map"
#define ADTF_FILTER_DESC "HTWK Occ Grid Map"

// Minimum and maximum distance in which the sensor data should be filtered
#define USONIC_MAX_DISTANCE_PROPERTY "maximum distance"
#define USONIC_MAX_DISTANCE_DEFAULT 2.5
#define USONIC_MAX_DISTANCE_MIN 0.5
#define USONIC_MAX_DISTANCE_MAX 4
#define USONIC_MAX_DISTANCE_DESCRIPTION "maximum distance of sensor in grid map"

#define USONIC_MIN_DISTANCE_PROPERTY "minimum distance"
#define USONIC_MIN_DISTANCE_DEFAULT 0.03
#define USONIC_MIN_DISTANCE_MIN 0.01
#define USONIC_MIN_DISTANCE_MAX 3
#define USONIC_MIN_DISTANCE_DESCRIPTION "minimum distance of sensor in grid map"

// Width and height of the openCV Mat
#define GRID_MAP_WIDTH 300
#define GRID_MAP_HEIGHT 300

// Distance in cm from each sensor to IMU
#define USONIC_MIDREAR_Y 0
#define USONIC_MIDREAR_X (-0.07)
#define USONIC_LEFTREAR_Y 0.06
#define USONIC_LEFTREAR_X (-0.11)
#define USONIC_RIGHTREAR_Y (-0.06)
#define USONIC_RIGHTREAR_X (-0.11)
#define USONIC_RIGHTMID_Y (-0.15)
#define USONIC_RIGHTMID_X 0.3
#define USONIC_LEFTMID_Y 0.15
#define USONIC_LEFTMID_X 0.3
#define USONIC_MIDFRONT_Y 0
#define USONIC_MIDFRONT_X 0.53
#define USONIC_MIDRIGHTFRONT_Y (-0.065)
#define USONIC_MIDRIGHTFRONT_X 0.52
#define USONIC_MIDLEFTFRONT_Y 0.065
#define USONIC_MIDLEFTFRONT_X 0.52
#define USONIC_RIGHTFRONT_Y (-0.11)
#define USONIC_RIGHTFRONT_X 0.51
#define USONIC_LEFTFRONT_Y 0.11
#define USONIC_LEFTFRONT_X 0.51

#define PI 3.14159265358979323846f

// Orientation of the sensor in degree (angle car - sensor)
#define USONIC_MIDREAR_HEADING 180
#define USONIC_LEFTREAR_HEADING 125
#define USONIC_RIGHTREAR_HEADING (-135)
#define USONIC_RIGHTMID_HEADING (-90)
#define USONIC_LEFTMID_HEADING 90
#define USONIC_MIDFRONT_HEADING 0
#define USONIC_MIDRIGHTFRONT_HEADING (-30)
#define USONIC_MIDLEFTFRONT_HEADING 30
#define USONIC_RIGHTFRONT_HEADING (-75)
#define USONIC_LEFTFRONT_HEADING 75

// Log odds values with which the occupancy of each grid is calculated
#define L_O 0

#define L_FREE_PROPERTY "free log odds"
#define L_FREE_DEFAULT (-0.6)
#define L_FREE_MIN (-1.0)
#define L_FREE_MAX (-0.1)
#define L_FREE_DESCRIPTION "log odds for free cells"

#define L_OCC_PROPERTY "occupied log odds"
#define L_OCC_DEFAULT 0.6
#define L_OCC_MIN 0.1
#define L_OCC_MAX 1.0
#define L_OCC_DESCRIPTION "log odds for occupied cells"

// Thickness of obstacles in m
#define ALPHA_PROPERTY "obstacle thickness"
#define ALPHA_DEFAULT 0.3
#define ALPHA_MIN 0.1
#define ALPHA_MAX 0.6
#define ALPHA_DESCRIPTION "Thickness of obstacles in m"

// opening angle of sensor beam in degree
#define BETA_PROPERTY "sensor opening angle"
#define BETA_DEFAULT 25
#define BETA_MIN 10
#define BETA_MAX 35
#define BETA_DESCRIPTION "opening angle of sensor beam in degree"

#define L_TRACKED_OBJECTS 1;

#define LOG_GRID_PROPERTY "Grid to CSV"

struct ultrasonicSensorPose{
    HTWKPoint pos;
    tFloat32 orientation=0;
    tFloat32 sensorValue=0;
};

enum ultrasonicSensors {
    USONICMIDREAR,
    USONICRIGHTREAR,
    USONICLEFTREAR,
    USONICRIGHTMID,
    USONICLEFTMID,
    USONICMIDFRONT,
    USONICMIDRIGHTFRONT,
    USONICMIDLEFTFRONT,
    USONICRIGHTFRONT,
    USONICLEFTFRONT,
    USONICSENSORCOUNT
};

class GridMaps: public adtf::cFilter {
ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC, adtf::OBJCAT_Auxiliary);

public:
    GridMaps(const tChar *__info);

    ~GridMaps();

private:
    //property values
    tFloat32 uSonicMaxDistance;
    tFloat32 uSonicMinDistance;
    tFloat32 lfree;
    tFloat32 locc;
    tFloat32 alphaValue;
    int betaValue;

    tBool logToCSV = tFalse;
    int csvCounter = 0;

    bool positionInit = false;
    bool debugThisCell = false;

    float occMap [GRID_MAP_WIDTH][GRID_MAP_HEIGHT];
    cv::Mat gridMap;

    tFloat32 heading;
    tFloat32 pos_x;
    tFloat32 pos_y;

    cVideoPin videoPin;
    tBitmapFormat outputBitmapFormat;
    cInputPin uSonicStructInputPin;
    cInputPin positionInputPin;
    cInputPin obstacleInput;

    tUltrasonicStruct receivedUSonicStruct;

    cObjectPtr<IMediaType> ultrasonicStructMediaType;
    cObjectPtr<IMediaTypeDescription> ultrasonicStructDescription;

    cObjectPtr<IMediaType> positionMediaType;
    cObjectPtr<IMediaTypeDescription> positionDescription;

    std::vector<tTrackingData> obstacleDataList;

    tResult Init(tInitStage eStage, ucom::IException **__exception_ptr);

    tResult CreateDescriptions(IException **__exception_ptr);

    tResult CreateOutputPins(IException **__exception_ptr);

    //tResult Stop(ucom::IException **__exception_ptr);

    //tResult Shutdown(tInitStage eStage, __exception);

    std::vector<ultrasonicSensorPose> sensorData;

    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

    // Calculates the sensor position and orientation respective to the given car position in world coordinates
    std::vector<ultrasonicSensorPose> ultrasonicSensorPoseCalculation(tFloat32 heading, tFloat32 posX, tFloat32 posY, tUltrasonicStruct sensorValues);

    // Helper method to rotate the sensor position around the IMU position
    void rotateSensorPoint(HTWKPoint &sensorPoint, const float &orientation, const HTWKPoint &IMUPosition);

    // Loops through occMap and calculates the occupancy for the nearest grid cells
    void updateGridMap(const std::vector<ultrasonicSensorPose> &sensorPos, tFloat32 carX, tFloat32 carY);

    // Checks for nearest sensor to input grid cell and evaluates occupancy with respective sensor data
    double inverseSensorModel(HTWKPoint gridCellCenter, std::vector<ultrasonicSensorPose> sensorPos);

    // Transforms the log odds occupancy value to a grey value
    int grayGridMapValue(double occValue);

    // Updates the gray values in the gridMap based on the new values given in the occMap
    void updateGrayValues();

    // Awesome magic methods to view the grid map in adtf
    tResult processVideo(cv::Mat &image);

    void writeCSV(string filename, cv::Mat m);
};


#endif //AADC_USER_GRIDMAPS_H
