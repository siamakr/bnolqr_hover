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
  uint8_t newDistance = 0;
  uint16_t  distance;
  
  if (garmin.getBusyFlag() == 0)
  {
      // Trigger the next range measurement
      garmin.takeRange();

      // Read new distance data from device registers
      distance = garmin.readDistance();

      // Report to calling function that we have new data
      newDistance = 1;
  }
  data.zb = distance/100.00f;
}

void Sensors::sampleBno(void)
{
  
  /////////EULER ANGLE////////// 
  pdata.roll = data.roll; 
  pdata.pitch = data.pitch; 
  pdata.yaw = data.yaw; 
  imu::Quaternion q = bno.getQuat();
  q.normalize();
  //convert quaternions into readable Euler angles in RADIANS 
  //given conversion from quaternions, the angles will automatically be in radians
  imu::Vector<3> euler = q.toEuler();
  //load data struct and switch z and x axes (still not sure why this is like this)
  data.roll = isnan(euler.z()) ? pdata.roll : euler.z();
  data.pitch = isnan(euler.y()) ? pdata.pitch : euler.y();
  data.yaw = isnan(euler.x()) ? pdata.yaw : euler.x();// - yaw_offset;

  //error catcher
  if( isnan(euler.z()) || isnan(euler.y()) || isnan(euler.x()) ) Serial.println("bnoerrored"); 
  //data.yaw -= yaw_offset;
  // data.roll = euler.z();
  // data.pitch = euler.y();
  // data.yaw = euler.x();

  //////////GYROSCOPE///////// 
  //load previous gyro data before retreiving the current data
  pdata.gx = data.gx; 
  pdata.gy = data.gy; 
  pdata.gz = data.gz;
  //read gyro values from bno
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  //load temp vecotor to IIR filter the gyro values 
  data.gx = IIR(d2r*gyro.x(), pdata.gx, 0.10);
  data.gy = IIR(d2r*gyro.y(), pdata.gy, 0.10); 
  data.gz = IIR(d2r*gyro.z(), pdata.gz, 0.55);

  //////////ACCELERATION/////////
  pdata.axb = data.axb; 
  pdata.ayb = data.ayb; 
  pdata.azb = data.azb;
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  data.axb = IIR(accel.x(), pdata.axb, .05);
  data.ayb = IIR(accel.y(), pdata.ayb, .05);
  data.azb = IIR(accel.z(), pdata.azb, .05);
}

void Sensors::runEstimator(void)
{
  float atemp[2] = {0}; 
  float ltemp[2] = {0};

  //save previous values to prev struct
  pdata.axw = data.axw; 
  pdata.ayw = data.ayw;
  pdata.azw = data.azw;

  pdata.vx = data.vx; 
  pdata.vy = data.vy; 
  pdata.vzi = data.vzi; 
  pdata.vzl = data.vzl;

  pdata.x = data.x; 
  pdata.y = data.y; 
  pdata.z = data.z;

  atemp[0] = data.axb; 
  atemp[1] = data.ayb; 
  atemp[2] = data.azb;
  rotate_to_world( atemp ); 

  //load current values in to current_data_t struct
  data.axw = atemp[0];
  data.ayw = atemp[1];
  data.azw = atemp[2];

  //filter and differentiate in one step. 
  data.vx =  IIR(DT_SEC * (atemp[0] - pdata.axw), pdata.vx, .05);
  data.vy =  IIR(DT_SEC * (atemp[1] - pdata.ayw), pdata.vy, .05);
  data.vzi =  IIR(DT_SEC * (atemp[2] - pdata.azw), pdata.vzi, .05);

  data.x = IIR((data.vx - pdata.vx) * DT_SEC, pdata.x, 0.05);
  data.y = IIR((data.vy - pdata.vy) * DT_SEC, pdata.y, 0.05);
  //data.x = IIR((data.vx - pdata.vx) * DT_SEC, pdata.x, 0.05);

  // rotate lidar measurement to world and integrate for vz_idar 
  ltemp[2] = data.zb; 
  rotate_to_world( ltemp ); 
  data.z = ltemp[2]; 

  // calculate vzworld from lidar measurement 
  data.vzl = IIR((data.z - pdata.z) / DT_MSEC, pdata.vzl, 0.05); 
}

float Sensors::IIR(float newSample, float prevOutput, float alpha)
{
  return ( (1.0f-alpha)*newSample + alpha * prevOutput);
}


///////////////// Private Functions Begin /////////////////



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


float Sensors::IIR( float newSample, float prevOutput, float alpha)
{
  return ( (1.0f-alpha)*newSample + alpha * prevOutput);
}




///////////////// Private Functions End /////////////////