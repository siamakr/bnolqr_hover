//// Controller.cpp /////

#include "Controller.h"


Servo sx; 
Servo sy; 
Servo rw; 
Servo edf; 

Controller::Controller()
{
  //attach servo pins 
  sx.attach(XSERVO_PIN);
  sy.attach(YSERVO_PIN);
  rw.attach(RW_PIN);
  edf.attach(EDF_PIN, 900, 2200);
  delay(200);
}

void Controller::init(void)
{
  //Zero Servos 
  writeXservo(0);
  writeYservo(0);
  delay(200);

  initEdf();              //comment out to not initiate servo startup seq.

}

void Controller::control_attitude_hov(float r, float p, float y, float gx, float gy, float gz, float z, float vz)
{
  //save previous control values for filtering 
  float u0, u1, u2, u3; 
  u0 = U_hov(0);
  u1 = U_hov(1);
  u2 = U_hov(2);
  u3 = U_hov(3);
  //load state vecotr 
  Xs_hov = {r, p, 0, gx, gy, 0, 0, 0};

  //run controller 
  error_hov = Xs_hov-REF_hov; 
  U_hov = -K_hov * error_hov; 
  //U_hov(3) += MASS * G; 
  U_hov(3) = MASS * G;

 

  //load desired torque vector
  //  float tx = U(2)*sin(U(0))*COM_TO_TVC;
  //  float ty = U(2)*sin(U(1))*COM_TO_TVC;
  //  float tz = U(2);

  /*
  //load new Thrust Vector from desired torque
  float Tx{ -U_hov(3) * sin(U_hov(0)) }; 
  float Ty{ -U_hov(3) * sin(U_hov(1)) * cos(U_hov(0)) }; 
  float Tz{  U_hov(3) * cos(U_hov(1)) * cos(U_hov(0)) };           //constant for now, should be coming from position controller 
  */

  float Tx{  U_hov(3) * sin(U_hov(0)) }; 
  float Ty{  U_hov(3) * sin(U_hov(1)) * cos(U_hov(0))  }; 
  float Tz{  U_hov(3)  };           //constant for now, should be coming from position controller 

  float Tm = sqrt(pow(Tx,2) + pow(Ty,2) + pow(Tz,2)); 
  U_hov(3) = Tm; 
  
  //different way to deduce servo angles from body forces (got this from a paper I can send you)
  //pdata.Roll = asin(-Tx/(Tmag - pow(Ty,2)));
  //pdata.Pitch = asin(Ty/Tmag);

  //U_hov(0) = asin(Tx/(Tm));
 // U_hov(1) = asin(Ty/Tm);

  //filter servo angles, the more filtering, the bigger the delay 
  U_hov(0) = IIR(U_hov(0), u0, .05);
  U_hov(1) = IIR(U_hov(1), u1, .05); 

  //limit servo angles to +-15º
  U_hov(0) = limit(U_hov(0), d2r(-15), d2r(15));
  U_hov(1) = limit(U_hov(1), d2r(-15), d2r(15)); 
  U_hov(3) = limit(U_hov(3), 15.00f, 27.00f);

  //write the actuation angles to the servos 
  writeXservo(-r2d(U_hov(0)));
  writeYservo(-r2d(U_hov(1)));
  writeEDF(U_hov(3));

  
  //load previous data struct for filtering etc. 
  // data.u1 = U_hov(0);
  // data.u2 = U_hov(1);
  // data.u3 = U_hov(2);
  // data.u4 = U_hov(3);
  // data.Tz = Tz; 
  // data.Tmag = Tm; 
  // data.ang1 = asin(Tx/Tm); 
  // data.ang2 = asin(Ty/Tm);
}



//////////////////// Private Functions Begin /////////////////////
void Controller::initEdf(void)
{
  //initialize the edf motor and run it at 1500us for 5 seconds to initialize the govenor mode to linearize the throttle curve. 
  edf.writeMicroseconds(EDF_OFF_PWM); 
  delay(1000);

  //go to 1500 and wait 5 seconds
  edf.writeMicroseconds(EDF_MIN_PWM);
  delay(5000);   
}

//Write to X servo 
void writeXservo(float angle)
{
  //map angle in degrees to pwm value for servo 
  int pwmX{round( X_P1 * pow(angle,2) + X_P2 * angle + X_P3 ) }; 
  sx.writeMicroseconds(pwmX); 
}

//write to Y servo 
void writeYservo(float angle)
{
  //map angle in degrees to pwm value for servo 
  int pwmY{ round(Y_P1 * pow(angle,2) + Y_P2 * (angle) + Y_P3 ) };      // using polynomial regression coefficients to map tvc angle to pwm vals
  // int pwmY{ round(Y_P1 * pow(angle,2) - Y_P2 * (angle) + Y_P3 ) };      // true regression equations
  sy.writeMicroseconds(pwmY); 
}

void writeEDF(float Ft)
{
  float omega{(Ft - RAD2N_P2)/RAD2N_P1}; 
  int pwm{round(omega * RAD2PWM_P1 + RAD2PWM_P2)}; 
  edf.writeMicroseconds(pwm);
}

float Controller::limit(float value, float min, float max)
{
  if(value >= max ) value = max; 
  if(value <= min ) value = min; 

  return value; 
}

float Controller::IIR( float newSample, float prevOutput, float alpha)
{
  return ( (1.0f-alpha)*newSample + alpha * prevOutput);
}

float Controller::r2d(float rad)
{
  return rad * 180.00f / PI;
}

float Controller::d2r(float deg)
{
  return deg * PI / 180.00f;
}



//////////////////// Private Functions End /////////////////////