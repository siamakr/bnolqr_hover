//// Sensors.h /////

#ifndef _EDFH_SENSORS_H
#define _EDFH_SENSORS_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_SPIDevice.h>
#include <LIDARLite_v3HP.h>
#include <BasicLinearAlgebra.h>
//#include "Sensors.h"


using namespace BLA;

//lidar stuff
uint8_t garmin_address{0x62};
uint16_t distance;
uint8_t lidar_measurement_response;

#define STATUS_READY 0x01
#define STATUS_FAILED_SETUP 0x02
#define STATUS_NO_RESPONSE 0x03

#define DT_MSEC 5.00f
#define DT_USEC DT_MSEC*1000.00f
#define DT_SEC DT_MSEC/1000.00

#define SENSOR_LIDAR_OFFSET 0.08f

LIDARLite_v3HP garmin;                       // Garmin v3HP LIDAR object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);  // BNO055 9-axis IMU object 

typedef struct
{
float roll, pitch, yaw; 
float gx, gy, gz;
float axb, ayb, azb;
float axw, ayw, azw;
float vx, vy, vzi, vzl, vz; 
float x, y, zb, z;
float u1, u2, u3, u4;
float Tmag, Tx, Ty, Tz; 
float e[7] = {0};         // This will be removed later, only for testing 
} sensor_data_t;

typedef struct __attribute__ ((packed)){
    float x, y, z;
    float vx, vy, vz;
} estimator_data_t; // 24 bytes

typedef struct{
    uint8_t imu;
    uint8_t flow;
    uint8_t lidar;
} sensor_status_t;

const float r2d{180.00f/PI}; 
const float d2r{PI/180.00f};

class Sensors
{
public:

    Sensors( void );
    
    // Init the sensor objects
    void init( void );

    void calibratebno(adafruit_bno055_offsets_t bnoOffset);

    float IIR( float newSample, float prevOutput, float alpha);

    void sampleBno(void);

    void sampleLidar(void);

    void runEstimator(void);



    sensor_data_t data;             // current data struct
    sensor_data_t pdata;            // this will be removed (for testing only)
    estimator_data_t estimate;      // estimate data struct
    sensor_status_t status;

private:

    adafruit_bno055_offsets_t bnoOffset;

    

    void rotate_to_world( float * vector );

    


};


#endif