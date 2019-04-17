//Task: Demonstrate multi-stepper functionality
#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_StepperMotor *leftStep = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *rightStep = AFMS.getStepper(200, 1);

void forwardStepLeft(){
  leftStep->onestep(FORWARD, DOUBLE);
  //leftStep->quickstep(FORWARD);  
}

void backwardStepLeft(){
  leftStep->onestep(BACKWARD, DOUBLE);
  //leftStep->quickstep(BACKWARD);  
}

void forwardStepRight(){
  rightStep->onestep(FORWARD, DOUBLE);
  //rightStep->quickstep(FORWARD);  
}

void backwardStepRight(){
  rightStep->onestep(BACKWARD, DOUBLE);
  //rightStep->quickstep(BACKWARD);  
}

AccelStepper leftStepper(forwardStepLeft, backwardStepLeft);
AccelStepper rightStepper(forwardStepRight, backwardStepRight);

void setup() {
  AFMS.begin();
  //rightStep->quickstepInit();
  //leftStep->quickstepInit();
  //TWBR = ((F_CPU /400000l) - 16) / 2; // Change the i2c clock to 400KHz
  //TWBR = 12;
  //Serial.begin(9600);

  leftStepper.setMaxSpeed(1000.0);
  //leftStepper.setAcceleration(1000.0);
  //leftStepper.moveTo(-50.0);
  leftStepper.setSpeed(-200.0);

  rightStepper.setMaxSpeed(1000.0);
  //rightStepper.setAcceleration(1000.0);
  //rightStepper.moveTo(50.0);
  rightStepper.setSpeed(200.0);
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
