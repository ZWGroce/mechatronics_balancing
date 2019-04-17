//Task: Create a self balancing robot
#include <Wire.h>  //Include library for i2c communication
#include <Adafruit_MotorShield.h>  //Include library to use motorshield
#include <AccelStepper.h>  //Include library to drive both stepers at the same time
#include <MPU6050.h>  //Include library to use MPU-6050 to measure angle
#include <FOLPF.h>  //Include library to filter dat
#include <Simple_PID.h>  //Include library to implement PID controller

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x60);  //Create motorshield object at default address 
Adafruit_StepperMotor *leftStep = AFMS.getStepper(200, 2);  //Create a stepper motor object for the left motor
Adafruit_StepperMotor *rightStep = AFMS.getStepper(200, 1);  //Create a stepper motor object for the right motor

void forwardStepLeft(){  //Create a wrapper for a left motor forward step 
  leftStep->onestep(FORWARD, DOUBLE);  //Move the left motor forward one step
}

void backwardStepLeft(){  //Create a wrapper for the left motor backward step
  leftStep->onestep(BACKWARD, DOUBLE);  //Move the left motor back one step
}

void forwardStepRight(){  //Create a wrapper for the right motor forward step
  rightStep->onestep(FORWARD, DOUBLE);  //Move the right motor forward one step
}

void backwardStepRight(){  //Create a wrapper for the right motor backward step
  rightStep->onestep(BACKWARD, DOUBLE);  //Move the right motor backward one step
}

AccelStepper leftStepper(forwardStepLeft, backwardStepLeft);  //Create a left motor object for the accelstepper library using the wrappers created above
AccelStepper rightStepper(forwardStepRight, backwardStepRight);  //Create a right motor object for the accelstepper library using the wrappers created above

//*****Define Variables*****
int start = 0;  //Variable to track whether or not the motors should run
double motorSpeed = 0.0;  //Tracks the speed the motors should be running at
double balancePoint = 29.5;  //Angle the robot should be at to be perfectly balanced

//MPU-6050 Variables
int ax, ay, az, gx, gy, gz;  //Create variables to measure the acceleration and gyro in each direction
double axf, ayf, azf;  //Creates variables to hold the filtered acceleration values
double rollA, pitchA, rollG, pitchG, roll, pitch, rollAccelBias, pitchAccelBias;
/*
 *rollA = track the roll angle measured from the accelerometer
 *pitchA = track the pitch angle measured from the accelerometer
 *rollG = track the roll angle measured from the gyro
 *pitchG = track the pitch angle measured from the gyro
 *roll = track the complementary filtered roll angle
 *pitch = track the complementary filtered pitch angle
 *rollAccelBias = holds the accel offset roll angle
 *pitchAccelBias = holds the accel offset pitch angle
 */
long gxb, gyb, gzb;  //tracks the gyro value for the program
double t, tlast, dt;  //tracks the time, last time, and change in time
double ts = 10.0;  //Sampling time (in milliseconds)
double gyroScale = 131.0;  //Calculation factor (LSB to angle)
double calibration = 200.0;  //Number of iterations the calibration loop should run

MPU6050 sensor;  //Creates an object to use the MPU-6050 sensor

//Filters
double alpha = 0.1;  //Set the filter variable value (lower value, more filtered, more delayed)
FOLPF filter = FOLPF(alpha);  //Creates a filter object using the alpha value

//PID
double kp = 6.0, ki = 0.0, kd = 4.0;  //Initialize P, I, and D gains
double uleft, uright, du;  //Track the set speed of the left and right motors, and the change in speed (output by the controller)
double duMax = 50.0, uMax = 150.0;  //Constrain the change in speed and the maximum top speed
double lastAngle;  //Track the past pitch angle

Simple_PID controller = Simple_PID(kp, ki, kd);  //Create an object of the PID controller using the kp, ki, and kd gains

//*****Program Initialization*****
void setup() {
  Wire.begin();  //Initialize for communication with i2c devices
  AFMS.begin();  //Initialze for use of the adafruit motor shield
  sensor.initialize();  //Initialize the MPU-6050
  //Serial.begin(9600);
  
  leftStepper.setMaxSpeed(uMax);  //Set the maximum speed of the left motor
  leftStepper.setSpeed(0.0);  //Set the left motor speed to 0
  rightStepper.setMaxSpeed(uMax);  //Set the maximum speed of the right motor
  rightStepper.setSpeed(0.0);  //Set right motor speed to 0

  sensor.setFullScaleAccelRange(0);  //Set the Accel range to +/- 2g
  sensor.setFullScaleGyroRange(0);  //Set the gyro range to +/- 250 deg/s

  for(int i = 0; i < calibration; i++){  //Run the inital calibration 200 times (@ 5ms = 1 second)
    sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //Pull readings from the MPU-6050

    axf = filter.getFilter(ax, axf);  //Filter the x accel value
    ayf = filter.getFilter(ay, ayf);  //Filter the y accel value
    azf = filter.getFilter(az, azf);  //Filter the z accel value

    rollAccelBias += atan2(-ayf, azf) * 180 / M_PI;  //Calculate the roll angle using Eq.2 of lab manual page 137
    pitchAccelBias += atan2(axf, sqrt((ayf * ayf) + (azf * azf))) * 180 / M_PI;  //Calculate the pitch angle using Eq.1
    
    gxb += gx;  //Increment the x gyro value
    gyb += gy;  //Increment the y gyro value
    gzb += gz;  //Increment the z gyro value
    delay(5);  //Delay the calibration loop by 5 milliseconds
  }

  rollAccelBias /= calibration;  //Divide the accel roll bias to get the average
  pitchAccelBias /= calibration;  //Divide the accel pitch bias to get the average
  
  gxb /= calibration;  //Average the gyro x bias
  gyb /= calibration;  //Average the gyro y bias
  gzb /= calibration;  //Average the gyro z bias
}

//*****Main Program*****
void loop() {
  sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //Pull readings from the MPU-6050
  axf = filter.getFilter(ax, axf);  //Filter the x accel value
  ayf = filter.getFilter(ay, ayf);  //Filter the y accel value
  azf = filter.getFilter(az, azf);  //Filter the z accel value
  
  rollA = (atan2(-ayf, azf) * 180 / M_PI) - rollAccelBias;  //Calculate the roll angle using Eq.2 of lab manual page 137
  pitchA = (atan2(axf, sqrt((ayf * ayf) + (azf * azf))) * 180 / M_PI) - pitchAccelBias;  //Calculate the pitch angle using Eq.1

  t = millis();  //Determine the current time
  dt = t - tlast;  //Find the time since the last 
  
  if(dt >= ts){  //Run the following code (angle determination and controller) if 10 ms (the sampling time) has passed
    sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //Pull readings from the MPU-6050

    gx -= gxb;  //Correct the x gyro value for the bias
    gy -= gyb;  //Correct the y gyro value for the bias
    gz -= gzb;  //Correct the z gyro value for the bias

    rollG += (gx / gyroScale) * (ts/1000.0);  //Calculate the gyro roll angle using integration
    pitchG += (gy / gyroScale) * (ts/1000.0);  //Calculate the gyro pitch angle using integration

    roll = (0.9996 * rollG) + (0.0004 * -1 * rollA);  //Calculate the roll using a complementary filter to correct for gyro drift
    pitch = ((0.9996 * pitchG) + (0.0004 * -1.0 * pitchA));  //Calculate the pitch using a complementary filter to correct for gyro drift
    //Note: The gyro values produced are negative, but the accel is positive, so invert the accel to get a more accurate value
    pitch *= -1.0;  //Invert the pitch reading to get only positive angles
    
    if(pitch < 0){  //Triggers if the pitch is less than 0
      pitch = 0;  //Set the pitch equal to 0 if it is negative, this will only occur rarely at extremis for values that fall above the average bias
    }

    if(pitch > (balancePoint - 0.5) && pitch < (balancePoint + 0.5)){  //Triggers if the pitch is within half a degree of the balancing point
      start = 1;  //Sets the start variable equal to 1 (ie robot should run if it is upright)
    }else if(pitch < 5.0 || pitch > 50.0){  //Triggers if the pitch is less than 5 or greater than 50 (ie tipped over)
      leftStepper.setSpeed(0.0);  //Stop the left motor
      rightStepper.setSpeed(0.0);  //Stop the right motor
      start = 0;  //Set the start variable to 0
    }
    
    if(start == 1){  //Triggers the following controller code if the robot is upright
      du = controller.getDU(balancePoint, pitch, t, tlast, lastAngle);  //Utilize the Simple_PID library to calculate the du based on the gains and sensor readings
      du = constrain(du, -duMax, duMax);  //Prevents the du from getting too large
      if(du < 3.0 && du > -3.0) du = 0;  //Creates a deadband to prevent excessive jitter when the error (and therefore du) is very small
      lastAngle = pitch;  //Updates the last angle with the pitch
      motorSpeed += du;  //Changes the motor speed by du
      uleft = -motorSpeed;  //Sets uleft to the inverse of the current motor speed to get the right direction
      uleft = constrain(uleft, -uMax, uMax);  //Constrains uleft so it doesn't fall outside the speed range of the motor
      uright = motorSpeed;  //Sets uright to the current motor speed
      uright = constrain(uright, -uMax, uMax);  //Constrains uright so it doesn't fall outside the speed range of the motor

      leftStepper.setSpeed(uleft);  //Updates the left stepper motor speed
      rightStepper.setSpeed(uright);  //Updates the right stepper motor speed
    }

    tlast = t;  //Updates the last time tracker
  }
  
  if(start == 1){  //Triggers if the robot is upright and the motors should be running
    leftStepper.runSpeed();  //Runs the left stepper motor at the current set speed
    rightStepper.runSpeed();  //Rungs the right stepper motr at the current set speed
  }
}
