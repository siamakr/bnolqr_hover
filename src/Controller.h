// //// Controller.h /////

// #ifndef _EDFH_CONTROLLER_H
// #define _EDFH_CONTROLLER_H


// #include <utility/imumaths.h>
// #include <Servo.h>
// #include <Math.h>
// #include <BasicLinearAlgebra.h>
// #include <stdint.h>



// #define SERVO_MIN_TIMING 900
// #define SERVO_MID_TIMING 1500
// #define SERVO_MAX_TIMING 2100

// #define MOTOR_MIN_DSHOT 0
// #define MOTOR_MAX_DSHOT 1500 // No more is needed to lift aircraft.

// #define SETPOINT_MAX_Z 1.0f // Max altitude (meters)
// #define SETPOINT_MAX_ROLL 0.1f // Max roll (radians)
// #define SETPOINT_MAX_PITCH 0.1f // Max pitch (radians)

// // Maps the kRPM output of the controller to DSHOT values. This varies with voltage, thus should probably implement RPM controller at some point
// #define MOTOR_KRPM_TO_DSHOT 72.43f 
// #define CONTROL_LOOP_INTERVAL 0.005f

// typedef enum{
//     CONTROL_STATUS_STATIONARY = 0,
//     CONTROL_STATUS_FLYING,
//     CONTROL_STATUS_LANDING,
// } control_status_t; 

// typedef enum{
//     SETPOINT_X = 0,
//     SETPOINT_Y,
//     SETPOINT_Z,
//     SETPOINT_ROLL,
//     SETPOINT_PITCH,
//     SETPOINT_YAW
// } control_setpoint_t;

// using namespace BLA;

// typedef struct __attribute__ ((packed)){
//     float a1, a2, a3, a4;   // Servo angles
//     uint16_t dshot;            // Motor signal
// } control_signal_t;

// class Control
// {
// public:

//     Control();

//     void init();


// private:



// };


// #endif