//Task: Create a self balancing robot
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <MPU6050.h>
#include <FOLPF.h>
#include <Simple_PID.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *leftStepper = AFMS.getStepper(200,1);
Adafruit_StepperMotor *rightStepper = AFMS.getStepper(200, 2);

//*****Define Variables*****

//MPU-6050 Variables
int ax, ay, az, gx, gy, gz;
double axf, ayf, azf, axb, ayb, azb;
double rollA, pitchA, rollG, pitchG, roll, pitch;
long gxb, gyb, gzb;
double t, tlast, dt;
double ts = 10;
double gyroScale = 131;
double calibration = 200;
MPU6050 sensor;

//Filters
double alpha = 0.1;
FOLPF filter = FOLPF(alpha);

//PID
double kp = 0.1, ki = 0, kd = 0;
double du;
double ubias = 0, duMax = 10;
double lastAngle;
Simple_PID controller = Simple_PID(kp, ki, kd);

//*****Program Initialization*****
void setup() {
  Wire.begin();
  sensor.initialize();
  Serial.begin(9600);

  sensor.setFullScaleAccelRange(0);  // +/- 2g
  sensor.setFullScaleGyroRange(0);  // +/- 250 deg/s

  for(int i = 0; i < calibration; i++){
    sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    axb += ax;
    ayb += ay;
    azb += az;
    gxb += gx;
    gyb += gy;
    gzb += gz;
    delay(5);
  }

  axb /= calibration;  //Calculates gyro bias for the x, y, and z axis
  ayb /= calibration;
  azb /= calibration;
  gxb /= calibration;  //Calculates gyro bias for the x, y, and z axis
  gyb /= calibration;
  gzb /= calibration;
}

//*****Main Program*****
void loop() {
  sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  axf = filter.getFilter(ax, axf);
  ayf = filter.getFilter(ay, ayf);
  azf = filter.getFilter(az, azf);
  
  axf -= axb;
  ayf -= ayb;
  azf -= azb;
  
  rollA = (atan2(-ayf, azf) * 180 / M_PI);  //Eq.2 of lab manual page 137
  pitchA = (atan2(axf, sqrt((ayf * ayf) + (azf * azf))) * 180 / M_PI);  //Eq.1

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
    pitch = (0.9996 * pitchG) + (0.0004 * pitchA);

    //Serial.println(roll); Serial.print("\t"); Serial.println(pitch);
    //Serial.print(rollA); Serial.print("\t"); Serial.println(pitchA); Serial.println();

    du = controller.getDU(0, pitch, t, tlast, lastAngle);
    du = constrain(du, -duMax, duMax);
    lastAngle = pitch;
    
    tlast = t;
  }
}
