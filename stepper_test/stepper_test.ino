//Task: Demonstrate multi-stepper functionality
#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_StepperMotor *leftStep = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *rightStep = AFMS.getStepper(200, 2);

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

  leftStepper.setMaxSpeed(200.0);
  leftStepper.setAcceleration(100.0);
  leftStepper.moveTo(30.0);
  //leftStepper.setSpeed(200.0);

  rightStepper.setMaxSpeed(200.0);
  rightStepper.setAcceleration(100.0);
  rightStepper.moveTo(-30.0);
  //rightStepper.setSpeed(200.0);

  //TWBR = ((F_CPU /400000l) - 16) / 2; // Change the i2c clock to 400KHz
  TWBR = 12;
}

void loop() {
  if(leftStepper.distanceToGo() == 0){
    leftStepper.moveTo(-leftStepper.currentPosition());
    //leftStepper.setSpeed(200.0); 
  }

  if(rightStepper.distanceToGo() == 0){
    rightStepper.moveTo(-rightStepper.currentPosition());
    //rightStepper.setSpeed(200.0); 
  }

  leftStepper.run();
  rightStepper.run();
}
