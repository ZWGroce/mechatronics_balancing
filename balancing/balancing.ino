//Task: Create a self balancing robot
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <AccelStepper.h>
#include <MPU6050.h>
#include <FOLPF.h>
#include <Simple_PID.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x60);
Adafruit_StepperMotor *leftStep = AFMS.getStepper(200,1);
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

//*****Define Variables*****
int start = 0;
double rpm = 200.0;

//MPU-6050 Variables
int ax, ay, az, gx, gy, gz;
double axf, ayf, azf, axb, ayb, azb;
double rollA, pitchA, rollG, pitchG, roll, pitch, rollAccelBias, pitchAccelBias;
long gxb, gyb, gzb;
double t, tlast, dt;
double ts = 10.0;
double gyroScale = 131.0;
double calibration = 200.0;
MPU6050 sensor;

//Filters
double alpha = 0.1;
FOLPF filter = FOLPF(alpha);

//PID
double kp = 2.5, ki = 0.0, kd = 0.0;
double uleft, uright, du;
double duMax = 100.0;
double lastAngle;
Simple_PID controller = Simple_PID(kp, ki, kd);

//*****Program Initialization*****
void setup() {
  Wire.begin();
  sensor.initialize();
  Serial.begin(9600);

  leftStepper.setMaxSpeed(rpm);
  leftStepper.setSpeed(rpm);
  rightStepper.setMaxSpeed(rpm);
  rightStepper.setSpeed(rpm);

  sensor.setFullScaleAccelRange(0);  // +/- 2g
  sensor.setFullScaleGyroRange(0);  // +/- 250 deg/s

  for(int i = 0; i < calibration; i++){
    sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    axf = filter.getFilter(ax, axf);
    ayf = filter.getFilter(ay, ayf);
    azf = filter.getFilter(az, azf);

    rollAccelBias += atan2(-ayf, azf) * 180 / M_PI;  //Eq.2 of lab manual page 137
    pitchAccelBias += atan2(axf, sqrt((ayf * ayf) + (azf * azf))) * 180 / M_PI;  //Eq.1
    
    gxb += gx;
    gyb += gy;
    gzb += gz;
    delay(5);
  }

  rollAccelBias /= calibration;
  pitchAccelBias /= calibration;
  
  gxb /= calibration;  //Calculates gyro bias for the x, y, and z axis
  gyb /= calibration;
  gzb /= calibration;

  //Serial.print(gyb); Serial.print("\t"); Serial.println(pitchAccelBias);
}

//*****Main Program*****
void loop() {
  sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  axf = filter.getFilter(ax, axf);
  ayf = filter.getFilter(ay, ayf);
  azf = filter.getFilter(az, azf);
  
  rollA = (atan2(-ayf, azf) * 180 / M_PI) - rollAccelBias;  //Eq.2 of lab manual page 137
  pitchA = (atan2(axf, sqrt((ayf * ayf) + (azf * azf))) * 180 / M_PI) - pitchAccelBias;  //Eq.1

  t = millis();
  dt = t - tlast;
  
  if(dt >= ts){
    sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    gx -= gxb;
    gy -= gyb;
    gz -= gzb;

    rollG += (gx / gyroScale) * (ts/1000);
    pitchG += (gy / gyroScale) * (ts/1000);

    roll = (0.9996 * rollG) + (0.0004 * rollA);
    pitch = ((0.9996 * pitchG) + (0.0004 * pitchA)) + 27;

    if(pitch > -0.5 && pitch > 0.5) start = 1;

    //Serial.print(roll); Serial.print("\t"); Serial.println(pitch);
    //Serial.print(rollA); Serial.print("\t"); Serial.println(pitchA); Serial.println();
    if(start == 1){
      du = controller.getDU(0.0, pitch, t, tlast, lastAngle);
      //Serial.println(du);
      du = constrain(du, -duMax, duMax);
      lastAngle = pitch;
      uleft = leftStepper.currentPosition() + du;
      uright = rightStepper.currentPosition() + du;

      /*leftStepper.moveTo(uleft);
      leftStepper.setSpeed(rpm);
      rightStepper.moveTo(uright);
      rightStepper.setSpeed(rpm);*/
    }
    
    tlast = t;
  }
  
  //leftStepper.runSpeedToPosition();
  //rightStepper.runSpeedToPosition();
}
