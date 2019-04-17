//Task: Demonstrate multi-stepper functionality
#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_StepperMotor *leftStep = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *rightStep = AFMS.getStepper(200, 1);

void forwardStepLeft(){
  leftStep->onestep(FORWARD, DOUBLE);  
}

void backwardStepLeft(){
  leftStep->onestep(BACKWARD, DOUBLE);  
}

void forwardStepRight(){
  rightStep->onestep(FORWARD, DOUBLE);  
}

void backwardStepRight(){
  rightStep->onestep(BACKWARD, DOUBLE);  
}

AccelStepper leftStepper(forwardStepLeft, backwardStepLeft);
AccelStepper rightStepper(forwardStepRight, backwardStepRight);

void setup() {
  AFMS.begin();
  TWBR = 12;
  //Serial.begin(9600);

  leftStepper.setMaxSpeed(1000.0);
  //leftStepper.setAcceleration(1000.0);
  //leftStepper.moveTo(-50.0);
  leftStepper.setSpeed(-100.0);

  rightStepper.setMaxSpeed(1000.0);
  //rightStepper.setAcceleration(1000.0);
  //rightStepper.moveTo(50.0);
  rightStepper.setSpeed(100.0);

  //TWBR = 12;  //Changes the i2c clock to 400KHz
}

void loop() {
  /*if(leftStepper.distanceToGo() == 0){
    leftStepper.moveTo(-leftStepper.currentPosition());
    //leftStepper.setSpeed(500.0); 
  }

  if(rightStepper.distanceToGo() == 0){
    rightStepper.moveTo(-rightStepper.currentPosition());
    //rightStepper.setSpeed(500.0); 
  }*/

  leftStepper.runSpeed();
  rightStepper.runSpeed();

  //Serial.print(leftStepper.currentPosition()); Serial.print("\t"); Serial.println(rightStepper.currentPosition());
}
