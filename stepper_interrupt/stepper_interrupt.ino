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
  rightStepper.moveTo(30.0);
  //rightStepper.setSpeed(200.0);

  TWBR = 12; // Change the i2c clock to 400KHz

  cli();//stop interrupts

  //set timer2 interrupt at 20kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 20khz increments
  OCR2A = 255;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
 
  sei();//allow interrupts

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
}

ISR(TIMER2_COMPA_vect){
  leftStepper.run();
  rightStepper.run();
}
