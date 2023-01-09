///// Sensors.cpp /////

#include "Sensors.h"

Sensors::Sensors(void){
    //Start i2c bus
    Wire.begin(); 

    //// IMU Initialization ////
    bno.begin(OPERATION_MODE_NDOF);
    bno.setExtCrystalUse(false);
}

void Sensors::init(void)
{
    ///// Lidar Initialization /////
    garmin.configure(0, garmin_address); 
  ////////////////BNO onboard Hopper//////////////
  //acceleration 
  bnoOffset.accel_offset_x = -20;
  bnoOffset.accel_offset_y = -170;
  bnoOffset.accel_offset_z = -25;
  //mag 
  bnoOffset.mag_offset_x = 6435;
  bnoOffset.mag_offset_y = -2156;
  bnoOffset.mag_offset_z = 726;
  //gyro
  bnoOffset.gyro_offset_x = -4;
  bnoOffset.gyro_offset_y = 0;
  bnoOffset.gyro_offset_z = -1;
  //radii
  bnoOffset.accel_radius = 1000;
  bnoOffset.mag_radius = 806;
//  */
////////////////BNO onboard Hopper//////////////

/*
////////////////BNO offboard Hopper//////////////
  //acceleration 
  bnoOffset.accel_offset_x = 22;
  bnoOffset.accel_offset_y = -76;
  bnoOffset.accel_offset_z = -4;
  //mag 
  bnoOffset.mag_offset_x = 110;
  bnoOffset.mag_offset_y = 89;
  bnoOffset.mag_offset_z = -1069;
  //gyro
  bnoOffset.gyro_offset_x = -1;
  bnoOffset.gyro_offset_y = -1;
  bnoOffset.gyro_offset_z = 0;
  //radii
  bnoOffset.accel_radius = 1000;
  bnoOffset.mag_radius = 485;
////////////////BNO OFFboard Hopper//////////////
  delay(20);
  */

  /* Initialise the sensor */
  if (!bno.begin(OPERATION_MODE_NDOF))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while (1);
  }
  //Run the calibration loop to attain offsets 
  //calibratebno(bnoOffset);            //Comment this out if you have experimental values. 
  bno.setSensorOffsets(bnoOffset); 
  delay(500);

}


void Sensors::sampleLidar(void)
{

}

void Sensors::sampleBno(void)
{

}

void Sensors::runEstimator(void)
{

}

float Sensors::IIR(float newSample, float prevOutput, float alpha)
{

}

///////////////// Private Functions Begin /////////////////
void Sensors::limit(float value, float min, float max)
{

}


void Sensors::rotate_to_world( float * vector ){

    float p = data.roll;
    float q = data.pitch;
    float u = data.yaw;

    Matrix<3,1> in = { vector[0], vector[1], vector[2] };
    Matrix<3,1> out = {0,0,0};

    Matrix<3,3> R = {   cos(q)*cos(u), sin(p)*sin(q)*cos(u)-cos(p)*sin(u), cos(p)*sin(q)*cos(u)+sin(p)*sin(u) ,
                        cos(q)*sin(u), sin(p)*sin(q)*sin(u)+cos(p)*cos(u), cos(p)*sin(q)*sin(u)-sin(p)*cos(u) ,
                        -sin(q),       sin(p)*cos(q),                      cos(p)*cos(q)                      };

    out = R * in;

    vector[0] = out(0);
    vector[1] = out(1);
    vector[2] = out(2);

}





///////////////// Private Functions End /////////////////