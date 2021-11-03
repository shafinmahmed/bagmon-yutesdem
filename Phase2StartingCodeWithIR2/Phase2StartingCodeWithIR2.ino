//+++++++++++++++++++++++++++++++++++++ GYRO STUFF ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//Use ultrasonics to move correct motors to move at the same speed

/********************----Motor pin decleration ---*******************************************************************************/
const int MotorPinR = 7; // for motor A
const int MotorSpeedPinR = 6; // for motor A
//const int MotorBrakePinA = 9; // for motor A


const int MotorPinL = 4; // for motor B
const int MotorSpeedPinL = 5;// for motor B
//const int MotorBrakePinB = 8;// for motor B

const int CW  = HIGH;
const int CCW = LOW;
/******************************************************************************************************************************/


/********************----IR L pin  decleration ---*******************************************************************************/
double distanceL;
const int IRL = 12;
const int IRLRead = A0;
/******************************************************************************************************************************/


/********************----IR R pin  decleration ---*******************************************************************************/
double distanceR;
const int IRR = 13;
const int IRRRead = A1;
/******************************************************************************************************************************/





/********************----Ultrasonic F pin decleration ---******************************************************************************/
// defines pins numbers
const int trigpinF = 9;
const int echopinF = 10;
/******************************************************************************************************************************/





/********************----PID Constants decleration ---*******************************************************************************/
double prevError = 0.0;
double error = 0.0;
double iError = 0.0;
double dError = 0.0;

double kp = 2.75;   double ki = 0.0;   double kd = 2.8; //Adjust as needed

int PWMR = 255;
int PWML = 255;

double lastPWM_R; double lastPWM_L;




// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  // put your setup code here, to run once:
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  /*********************************************************************************************************************/

  Serial.begin(9600);//  seial monitor initialized


  /********************----IR L  Setup ---*******************************************************************************/
  pinMode(IRL, OUTPUT); //enable pin
  /**********************************************************************************************************************/

  /********************----IR R  Setup ---*******************************************************************************/
  pinMode(IRR, OUTPUT); //enable pin
  /**********************************************************************************************************************/




  /********************----Ultrasonic F Setup ---******************************************************************************/
  pinMode(trigpinF, OUTPUT); // Sets the trigpinF as an Output
  pinMode(echopinF, INPUT); // Sets the echopinF as an Input
  /**********************************************************************************************************************/




  /********************----Motor Setup ---*******************************************************************************/
  // motor A pin assignment
  pinMode(MotorPinR, OUTPUT);
  pinMode(MotorSpeedPinR, OUTPUT);

  // motor B pin assignment
  pinMode(MotorPinL, OUTPUT);
  pinMode(MotorSpeedPinL, OUTPUT);

  /****************************** GYRO SETUP *********************************************************************/

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
Serial.print("L: "); Serial.print(getDistanceLeft()); Serial.print("    R: "); Serial.println(getDistanceRight());
goStraight(millis);
}

void goStraight(long startTime) {

  if ((getDistanceRight() > 20.0) || (getDistanceLeft() > 20.0)) { //this is a redundancy. maybe we can remove?
    allMotorStop();
    return;
  }

  delay(10);

  error = getDistanceRight() - getDistanceLeft();

  Serial.print("ERROR: "); Serial.println(error);

  long newTime = millis();

  long elapsedTime = newTime - startTime;

  iError += error * elapsedTime;
  dError = (error - prevError) / (elapsedTime);

  double totalError = (kp * error) + (ki * iError) + (kd * dError);

  PWMR = 100 - totalError;
  PWML = 0.95*(100 + totalError) ;

  if (PWMR < 0) {
    PWMR = 0;
  }
  if (PWMR > 255) {
    PWMR = 255;
  }


  if (PWML < 0) {
    PWML = 0;
  }
  if (PWML > 255) {
    PWML = 255;
  }


  digitalWrite(MotorPinL, CCW);// set direction
  analogWrite(MotorSpeedPinL, PWML);// set speed at maximum

  digitalWrite(MotorPinR, CCW);// set direction
  analogWrite(MotorSpeedPinR, PWMR);// set speed at maximum

  lastPWM_R = PWMR;
  lastPWM_L = PWML;

}

void goStraight_RampDown(long startTime) {
  if ((getDistanceRight() > 30.0) || (getDistanceLeft() > 30.0)) { //this is a redundancy. maybe we can remove?
    allMotorStop();
    return;
  }

  delay(10);

  error = getDistanceRight() - getDistanceLeft();

  long newTime = millis();

  long elapsedTime = newTime - startTime;

  iError += error * elapsedTime;
  dError = (error - prevError) / (elapsedTime);

  double totalError = (kp * error) + (ki * iError) + (kd * dError);

  PWMR = 100 - totalError;
  PWML = 100 + totalError ;

  if (PWMR < 0) {
    PWMR = 0;
  }
  if (PWMR > 255) {
    PWMR = 255;
  }


  if (PWML < 0) {
    PWML = 0;
  }
  if (PWML > 255) {
    PWML = 255;
  }

  double PWML_Adjusted = PWML * 0.50;
  double PWMR_Adjusted = PWMR * 0.50;

  digitalWrite(MotorPinL, CCW);// set direction
  analogWrite(MotorSpeedPinL, PWML_Adjusted);// set speed at maximum

  digitalWrite(MotorPinR, CCW);// set direction
  analogWrite(MotorSpeedPinR, PWMR_Adjusted);// set speed at maximum

  lastPWM_R = PWMR;
  lastPWM_L = PWML;
}

double getDistanceLeft() {


  digitalWrite(IRL, HIGH);
  distanceL = analogRead(IRLRead);
  distanceL = (distanceL * -0.1499) + 69.793 ;
  delay(100);
  return distanceL;
}

double getDistanceRight() {

  digitalWrite(IRR, HIGH);
  distanceR = analogRead(IRRRead);
  distanceR = (distanceR * -0.1499) + 69.793 ;
  delay(100);
  return distanceR;
}

double getDistanceFront() {
  digitalWrite(trigpinF, LOW);
  delayMicroseconds(2);
  // Sets the trigpinF on HIGH state for 10 micro seconds
  digitalWrite(trigpinF, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpinF, LOW);
  // Reads the echopinF, returns the sound wave travel time in microseconds
  double durationF = pulseIn(echopinF, HIGH);
  // Calculating the distanceF
  double distanceF = durationF * 0.0343 / 2;

  return distanceF;
}


double getYawDeg() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    double retVal = (ypr[0] * 180 / M_PI);
    return retVal;
  }
}

void allMotorStop() {
  digitalWrite(MotorPinL, CCW);// set direction
  analogWrite(MotorSpeedPinL, 0);// set speed

  digitalWrite(MotorPinR, CCW);// set direction
  analogWrite(MotorSpeedPinR, 0);// set speed
}

void turnLeft90() {
  double yawInit = getYawDeg();
  double yawTarget = yawInit - 85;

  /////////////////////////// PULL BACK SLIGHTLY /////////////////////////////
  digitalWrite(MotorPinL, CW);// set direction
  analogWrite(MotorSpeedPinL, 50);// set speed
  digitalWrite(MotorPinR, CW);// set direction
  analogWrite(MotorSpeedPinR, 145);// set speed
  delay(750);

  while (1) {
    double theVal = getYawDeg();
    if (theVal < yawTarget) {
      allMotorStop();
      delay(500);

      digitalWrite(MotorPinL, CCW);// set direction
      analogWrite(MotorSpeedPinL, 95);// set speed

      digitalWrite(MotorPinR, CCW);// set direction
      analogWrite(MotorSpeedPinR, 145);// set speed


      delay(1250);

      allMotorStop();

      return;
    }
    else {
      digitalWrite(MotorPinL, CW);// set direction
      analogWrite(MotorSpeedPinL, 10);// set speed
      digitalWrite(MotorPinR, CCW);// set direction
      analogWrite(MotorSpeedPinR, 145);// set speed
    }
  }
}

void turnRight90() {

  /////////////////////////// PULL FORWARD SLIGHTLY //////////////////////////

  digitalWrite(MotorPinL, CCW);// set direction
  analogWrite(MotorSpeedPinL, 95);// set speed

  digitalWrite(MotorPinR, CCW);// set direction
  analogWrite(MotorSpeedPinR, 190);// set speed

  delay(400);


  /////////////////////////// PULL BACK SLIGHTLY /////////////////////////////
  digitalWrite(MotorPinL, CW);// set direction
  analogWrite(MotorSpeedPinL, 180);// set speed
  digitalWrite(MotorPinR, CW);// set direction
  analogWrite(MotorSpeedPinR, 25);// set speed
  delay(900);
  Serial.print(getYawDeg());
  double yawInit = getYawDeg();
  double yawTarget = yawInit + 85;

  while (1) {
    double theVal = getYawDeg();
    if (theVal > yawTarget) {
      allMotorStop();
      delay(500);

      digitalWrite(MotorPinL, CCW);// set direction
      analogWrite(MotorSpeedPinL, 95);// set speed

      digitalWrite(MotorPinR, CCW);// set direction
      analogWrite(MotorSpeedPinR, 145);// set speed

      delay(1500);

      allMotorStop();

      return;
    }
    else {
      digitalWrite(MotorPinL, CCW);// set direction
      analogWrite(MotorSpeedPinL, 145);// set speed
      digitalWrite(MotorPinR, CW);// set direction
      analogWrite(MotorSpeedPinR, 95);// set speed
    }
  }
}

void Reset_Gyro() {
  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}
