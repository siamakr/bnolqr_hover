//// Controller.h /////

#ifndef _EDFH_CONTROLLER_H
#define _EDFH_CONTROLLER_H


#include <Servo.h>
#include <Math.h>
#include <BasicLinearAlgebra.h>
#include <stdint.h>
#include <LIDARLite_v3HP.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_SPIDevice.h>

//// Teensy 4.1 Pin assignments
#define XSERVO_PIN 8
#define YSERVO_PIN 10
#define RW_PIN 11
#define EDF_PIN 9

//// Vehicle Specs + General Constants
#define COM_TO_TVC 0.1335
#define MASS 2.558                    //Kg
#define G 9.87

//// Vehicle's Minimums and Maximums 
#define MAX_ANGLE_SERVO 15.00f           //Deg
#define SERVO_MAX_SEC_PER_DEG 0.003333f      //dt/.003333 |(dt=0.1) = 3º
#define SERVO_MAX_SEC_PER_RAD 0.0095496f      //dt/.003333 |(dt=0.1) = 3º
#define SERVO_MAX_DEGREE_PER_DT 12
#define MAX_VEHICLE_ANGLE_DEG 35.00f
#define DEADBAND_ANGLE_DEG 0.001f
#define SERVO_ANG_TO_TVC_ANG 3.00f

//// Servo + EDF + Reaction Wheel Params 
#define EDF_OFF_PWM 900                 //uSec
#define EDF_MIN_PWM 1500                //uSec
#define EDF_MAX_PWM 2000                //uSec
#define EDF_MAX_SUSTAINED_PWM 1730      //uSec
#define EDF_IDLE_PWM 1600               //uSec
//VALUES SET FOR +-15º GIMBAL ANGLE FOR BOTH X AND Y
#define SERVO_INC_STEP_SIZE 10
#define SERVO_X_CENTER_US 1440
#define SERVO_Y_CENTER_US 1535
#define SERVO_X_MIN_US 1224
#define SERVO_Y_MIN_US 1160
#define SERVO_X_MAX_US 1892
#define SERVO_Y_MAX_US 1836

//SERVO-ANGLE TO TVC ANGLE POLYNOMIAL TRANSOFORMATOR 
#define X_P1 -29.2198405328854f 
#define X_P2 1453.88991021228f
#define Y_P1 -20.4054981741083f 
#define Y_P2 1530.81204806643f

//EDF MOTOR RPM TO THRUST TO PWM TRANSFORMATORS
#define RAD2N_P1 0.018566536813619f    //Newtons to radians/s 
#define RAD2N_P2 -22.506778362213f     //Newtons to Radians/s
#define RAD2PWM_P1 0.2719786528784f    //Radians/s to PWM(us)  
#define RAD2PWM_P2 1010.29617153703f   //Radians/s to PWM(us)
#define RPM_TO_OMEGA (2.0f*PI/60.0f)        //RPM to Radians/s
#define OMEGA_TO_RPM (60.0f/(2.0f*PI))      //Radians/s to RPM
#define GRAMS_TO_NEWTONS (9.80f / 1000.00f) //Grams(g) to Newtons(N)

//MASS-MOMENT-OF-INERTIA OF VEHICLE
#define V_JXX 0.0058595f
#define V_JYY 0.0058595f
#define V_JZZ 0.01202768f
#define EDF_JZZ 0.0001744f      //MASS-MOMENT-OF-INERTIA OF EDF-PROP/MOTOR
#define RW_JZZ 0.00174245f      //MASS-MOMENT-OF-INERTIA OF REACTION WHEEL



typedef enum{
    CONTROL_STATUS_STATIONARY = 0,
    CONTROL_STATUS_FLYING,
    CONTROL_STATUS_LANDING,
} control_status_t; 

typedef enum{
    SETPOINT_X = 0,
    SETPOINT_Y,
    SETPOINT_Z,
    SETPOINT_ROLL,
    SETPOINT_PITCH,
    SETPOINT_YAW
} control_setpoint_t;

using namespace BLA;



class Controller
{
public:

    Controller();

    void init();

    void control_attitude_hov(float r, float p, float y, float gx, float gy, float gz, float z, float vz);


private:

    void writeXservo(float angle);

    void writeYservo(float angle); 

    void initEdf(void);

    float limit(float value, float min, float max); 

    float IIR(float newSample, float prevOutput, float alpha);


Matrix<4,1> U_hov = {0.00,0.00,0.00,0.00}; // Output vector
Matrix<8,1> error_hov {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0}; // State error vector
Matrix<8,1> REF_hov = {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00}; 
Matrix<8,1> Xs_hov = {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};
Matrix<4,8> K_hov = {  -0.4164,   -0.0000,    0.0000,   -0.1440,   -0.0000,    0.0000,    0.0000,    0.0000,
                        0.0000,   -0.4164,    0.0000,   -0.0000,   -0.1440,   -0.0000,    0.0000,    0.0000,
                       -0.0000,    0.0000,   -0.0316,   -0.0000,    0.0000,   -0.0561,    0.0000,    0.0000,
                        0.0000,    0.0000,    0.0000,   -0.0000,    0.0000,    0.0000,   9999.6288,   12.0942}; 


};


#endif