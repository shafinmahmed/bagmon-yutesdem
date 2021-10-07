#include <Adafruit_VL53L0X.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

Adafruit_VL53L0X eye1 = Adafruit_VL53L0X();
MPU6050 mpu;

#define INTERRUPT_PIN 2

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector




double error, prevError, iError, dError;
double kp = 15.0; double ki = 0.0; double kd = 3.5;
const double maintainDist = 70.00;



/********************----Motor pin decleration ---*******************************************************************************/
const int MotorPinR = 7; // for motor A
const int MotorSpeedPinR = 6; // for motor A
//const int MotorBrakePinA = 9; // for motor A


const int MotorPinL = 4; // for motor B
const int MotorSpeedPinL = 5;// for motor B
//const int MotorBrakePinB = 8;// for motor B

const int CW  = HIGH;
const int CCW = LOW;




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


  Serial.begin(9600);

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



  if (!eye1.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
  }

  /********************----Motor Setup ---*******************************************************************************/
  // motor A pin assignment
  pinMode(MotorPinR, OUTPUT);
  pinMode(MotorSpeedPinR, OUTPUT);

  // motor B pin assignment
  pinMode(MotorPinL, OUTPUT);
  pinMode(MotorSpeedPinL, OUTPUT);
  /**********************************************************************************************************************/
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  long startTime = millis();
  long endTime = startTime + 10000;

  while(1) {
    goStraight(millis());
  }
  */
  if(getTOFRange() < 70.00) {
    analogWrite(MotorSpeedPinL, 255);
    analogWrite(MotorSpeedPinR, 0);
    Serial.println("HERE");
  }
  else {
    analogWrite(MotorSpeedPinL, 0);
    analogWrite(MotorSpeedPinR, 0);
  }
}

double getTOFRange() {
  double retVal = -1;

  VL53L0X_RangingMeasurementData_t measure;

  eye1.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    //Serial.print("Range: ");
    retVal = measure.RangeMilliMeter;
    //Serial.println(" mm");
  }
  delay(100);
  return retVal;
}

double getYawDeg() {
  double retVal = -1;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    retVal = (ypr[0] * 180 / M_PI);
  }
  return retVal;
}

void goStraight(long startTime) {
  //delay(100);
  error = getTOFRange() - maintainDist;
  long newTime = millis();
  long elapsedTime = newTime - startTime;

  iError += error * elapsedTime;
  dError = (error - prevError) / (elapsedTime);

  prevError = error;

  double totalError = ((kp * error) + (ki * iError) + (kd * dError));

  double PWM_adjustment;

  if (totalError > -350 && totalError < 450) {
    PWM_adjustment = map(totalError, -350, 450, -250, 250);
    if (PWM_adjustment > 0) { //move towards right
      analogWrite(MotorSpeedPinR, 255);
      analogWrite(MotorSpeedPinL, (255 - PWM_adjustment));
      Serial.print("Adjusting towards right ");
      Serial.println(255 - PWM_adjustment);
    }
    else { //move towards left
      analogWrite(MotorSpeedPinR, (255 + PWM_adjustment));
      analogWrite(MotorSpeedPinL, 255);
      Serial.print("Adjusting towards left ");
      Serial.println(255 + PWM_adjustment);
    }
  }  
}
