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





//////////////////// Private Functions End /////////////////////