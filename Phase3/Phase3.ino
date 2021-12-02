//+++++++++++++++++++++++++++++++++++++ GYRO STUFF ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "I2Cdev.h"
#include "Servo.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif



/********************----Servo pin decleration ---*******************************************************************************/
Servo servo_test;        //initialize a servo object for the connected servo
/******************************************************************************************************************************/




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
const int IRL = 12;
const int IRLRead = A0;
/******************************************************************************************************************************/


/********************----IR R pin  decleration ---*******************************************************************************/
const int IRR = 13;
const int IRRRead = A1;
/******************************************************************************************************************************/





/********************----Ultrasonic R pin decleration ---******************************************************************************/
// defines pins numbers
const int trigpinR = 9;
const int echopinR = 10;
/******************************************************************************************************************************/



/********************----Ultrasonic L pin decleration ---******************************************************************************/
// defines pins numbers
const int trigpinL = 3;
const int echopinL = 8;
/******************************************************************************************************************************/







/********************----PID Constants decleration ---*******************************************************************************/
double prevError = 0.0;
double error = 0.0;
double iError = 0.0;
double dError = 0.0;
// p = 6.3 d = 4.3
double kp = 6.3;   double ki = 0.0;   double kd = 4.8; //Adjust as needed

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
  /********************----Servo  Setup ---*******************************************************************************/
  //servo_test.attach(11);      // attach the signal pin of servo to pin 11 of arduino
  /**********************************************************************************************************************/





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




  /********************----Ultrasonic R Setup ---******************************************************************************/
  pinMode(trigpinR, OUTPUT); // Sets the trigpinR as an Output
  pinMode(echopinR, INPUT); // Sets the echopinR as an Input
  /**********************************************************************************************************************/


  /********************----Ultrasonic L Setup ---******************************************************************************/
  pinMode(trigpinL, OUTPUT); // Sets the trigpinL as an Output
  pinMode(echopinL, INPUT); // Sets the echopinL as an Input
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

  long startTime, timerOffset;
  double dL, dR;
  //servo_test.write(180);
  delay(100);





  //gripperUp();
  /*
    servo_test.detach();
    /*

    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }
    long timeNow = millis();
    bool stopCond = false;
    while (dL < 30.00 && dR < 30.00) { //this is a redundancy. maybe we can remove?
    goUp_BackwardsLong(millis());

    dL = UltrasonicRight();
    dR = UltrasonicRight();

    if (dR > 25.00) {
      dR = UltrasonicRight();
    }
    if (dL > 25.00) {
      dL = UltrasonicLeft();
    }
    }
    Serial.println("Exiting goStraight");
    allMotorStop();
    delay(500);

    burstFwd();
    delay(350);
    allMotorStop();
    delay(250);



  */

  /*


    ///////////// #1 SMALL STRAIGHT ////////////////////

    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }
    long timeNow = millis();
    bool stopCond = false;
    while (dL < 20.00 && dR < 20.00) { //this is a redundancy. maybe we can remove?
    goStraight(millis());

    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
      dR = UltrasonicRight();
    }
    if (dL > 25.00) {
      dL = UltrasonicLeft();
    }
    long endingTime = millis();

    if (endingTime > (timeNow + 10000)) {
      stopCond = true;
    }
    }
    Serial.println("Exiting goStraight");
    allMotorStop();
    delay(500);

    burstFwd();
    delay(350);
    allMotorStop();
    delay(250);


    ////////////////// #2 LEFT TURN /////////////////////////////

    Reset_Gyro();

    turnLeft90();

    allMotorStop();

    delay(500);


    /*



    ///////////////////// #3 LONG STRAIGHT /////////////////////////

    dL = getDistanceLeft();
    dR = getDistanceRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }

    while (dL < 20.0 && dR < 20.0) { //this is a redundancy. maybe we can remove?
    goStraight(millis());

    dL = getDistanceLeft();
    dR = getDistanceRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }
    }

    allMotorStop();
    delay(500);

    burstFwd();
    delay(350);
    allMotorStop();
    delay(250);




    /////////// #4 LEFT TURN /////////////////////


    //Reset_Gyro();

    turnLeft90();

    allMotorStop();

    delay(500);



    /////////////// #5 STRAIGHT /////////////

    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }

    while (dL < 12.0 && dR < 12.0) { //this is a redundancy. maybe we can remove?
    goStraight(millis());

    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }
    }

    allMotorStop();
    delay(500);

    burstFwd();
    delay(1150);
    allMotorStop();
    delay(250);





    //////// #6 RIGHT TURN /////////////////////

    Reset_Gyro();
    turnLeftBackwards90();

    allMotorStop();

    delay(500);



    /////////////// #7 BURST FORWARD ///////////////

    burstBkwd();
    delay(1150);
    allMotorStop();
    delay(500);


    /*

    //////////////////// #8 RIGHT TURN //////////////////

    Reset_Gyro();
    turnRight90();

    allMotorStop();

    delay(500);

    burstFwd_MDF();
    delay(500);
    allMotorStop();
    delay(500);


    ////////////////////// #9 SHORT RAMPDOWN ///////////////

    dL = getDistanceLeft();
    dR = getDistanceRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }

    while (dL < 20.0 && dR < 20.0) { //this is a redundancy. maybe we can remove?
    goStraight_BackwardsDownShort(millis());

    dL = getDistanceLeft();
    dR = getDistanceRight();

    if (dR > 25.00) {
      dR = UltrasonicRight();
    }
    if (dL > 25.00) {
      dL = UltrasonicLeft();
    }
    }

    allMotorStop();
    delay(500);

    burstFwd();
    delay(1000);
    allMotorStop();
    delay(250);

  */

  /*

    //////////////// #10 RIGHT TURN ////////////////////////////////

    Reset_Gyro();
    turnRight90_MidRamp();

    allMotorStop();

    delay(500);

    burstFwd_MDF();
    delay(1500);
    allMotorStop();
    delay(500);

  */

  /*
    ///////////////////////// #11 LONG RAMP DOWN ////////////////////

    long newStartTime = millis();
    long newEndTime = newStartTime + 20000;

    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }

    while (dL < 25.0 && dR < 25.0) { //this is a redundancy. maybe we can remove?
    long newTimeNow = millis();
    if (newTimeNow > newEndTime) {
    break;
    }
    goStraight_RampDownLong(millis());

    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }
    }

    allMotorStop();
    delay(500);

    burstFwd_MDF();
    delay(1000);
    allMotorStop();
    delay(250);

  */

  /*

    //////////// #12 RIGHT TURN //////////////////////

    Reset_Gyro();
    turnRight90();

    allMotorStop();

    delay(500);



    ////////////////////////// #13 STRAIGHT ///////////////
    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }

    while (dL < 25.0 && dR < 25.0) { //this is a redundancy. maybe we can remove?
    goStraight(millis());

    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }
    }

    allMotorStop();
    delay(500);
    burstFwd_MDF();
    delay(500);
    allMotorStop();
    delay(250);



    ///////////// #14 RIGHT TURN ///////////////////////////

    Reset_Gyro();
    turnRight90();

    allMotorStop();

    delay(500);



    ///////////////// #15 BURST FORWARD /////////////////////

    burstFwd();
    delay(1850);
    allMotorStop();
    delay(500);



    ///////////////////////// #16 RIGHT TURN ///////////////

    Reset_Gyro();
    turnRight90();

    allMotorStop();

    delay(500);

    burstFwd();
    delay(600);
    allMotorStop();



    ////////////////////////// #17 STRAIGHT ///////////////
    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }

    while (dL < 20.0 && dR < 20.0) { //this is a redundancy. maybe we can remove?
    goStraight(millis());

    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }
    }

    allMotorStop();
    delay(500);

    burstFwd();
    delay(350);
    allMotorStop();
    delay(250);



    ///////////////////////// #18 LEFT TURN ///////////////

    Reset_Gyro();
    turnLeft90();

    allMotorStop();

    delay(500);


    ////////////////////////// #19 STRAIGHT ///////////////
    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }

    while (dL < 20.0 && dR < 20.0) { //this is a redundancy. maybe we can remove?
    goStraight(millis());

    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }
    }

    allMotorStop();
    delay(500);

    burstFwd();
    delay(350);
    allMotorStop();
    delay(250);


    ////////////////////// #20  RIGHT FOLLOW STRAIGHT   (NEED TO TIME THIS)  //////////////////

    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }

    start = millis();
    timerOffset = 4000;
    while (dR < 20.00) {

    long ending = millis();

    if (ending > (start + timerOffset)) {
    break;
    }

    goStraight_R(millis());

    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }
    }
    allMotorStop();
    delay(500);

    burstFwd();
    delay(350);
    allMotorStop();
    delay(250);




    ////////////// #21 LEFT TURN //////////////////

    Reset_Gyro();
    turnLeft90();

    allMotorStop();

    delay(500);




    ///////////////// #22 STRAIGHT /////////////////////

    dL = getDistanceLeft();
    dR = getDistanceRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }

    while (dL < 20.0 && dR < 20.0) { //this is a redundancy. maybe we can remove?
    goStraight(millis());

    dL = getDistanceLeft();
    dR = getDistanceRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }
    }

    allMotorStop();
    delay(500);

    burstFwd();
    delay(350);
    allMotorStop();
    delay(250);



    ////////////// #23 LEFT TURN //////////////////

    Reset_Gyro();
    turnLeft90();

    allMotorStop();

    delay(500);






    ///////////////// #24 BURST FORWARD /////////////////////

    burstFwd();
    delay(1250);
    allMotorStop();
    delay(500);




    ////////////// #25 BACKWARDS LEFT TURN //////////////////

    Reset_Gyro();
    turnLeft270();

    allMotorStop();

    delay(500);


    //////////////////////// #26 BURST BACKWARDS ///////////////
    burstBkwd();
    delay(1250);
    allMotorStop();
    delay(500);

    //////// GRIPPER DOWN /////////

    servo_test.attach(11);
    gripperDown();
    delay(500);


    ////////////// #27 BACKWARDS STRAIGHT (NEED TO TIME THIS) ////////////////////
    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }

    if (dL > 50) {
    dL = 0;
    }
    if (dR = 50) {
    dR = 0;
    }

    start = millis();
    timerOffset = 6000;
    while (dR < 12.00 && dL < 12.00) { //this is a redundancy. maybe we can remove?

    //Serial.print("L: "); Serial.print(dL); Serial.print("    R: "); Serial.println(dR);

    long ending = millis();

  */

  /*
    if (ending > (start + timerOffset)) {
    break;
    }
  */


  /*

    goStraight_BackwardsDownShort(millis());

    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dL > 50) {
    dL = 0;
    }
    if (dR = 50) {
    dR = 0;
    }

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }
    }

    allMotorStop();
    delay(500);


    /*
    Reset_Gyro();
    turnLeft270();

    allMotorStop();

    delay(500);



  */

  /*

    servo_test.attach(11);
    gripperDown();




    delay(1500);



    servo_test.detach();





    ////////////// #27 BACKWARDS STRAIGHT (NEED TO TIME THIS) ////////////////////
    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }

    if (dL > 50) {
    dL = 0;
    }
    if (dR > 50) {
    dR = 0;
    }

    startTime = millis();
    timerOffset = 10000;
    while (1) { //this is a redundancy. maybe we can remove?

    Serial.print("L: "); Serial.print(dL); Serial.print("    R: "); Serial.println(dR);

    long ending = millis();
    if (dR > 15.00 || dL > 15.00) {
    burstBkwd_Fast();
    break;
    }

    if (ending > (startTime + timerOffset)) {
    burstBkwd_Fast();
    break;
    }



    goStraight_Backwards(millis());

    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dL > 50) {
    dL = 0;
    }
    if (dR = 50) {
    dR = 0;
    }

    if (dR > 25.00) {
    dR = UltrasonicRight();
    }
    if (dL > 25.00) {
    dL = UltrasonicLeft();
    }
    }






    ////////////////////  #28 PICKUP ACTION ////////////////////



    burstFwd();
    delay(350);
    allMotorStop();

    servo_test.attach(11);

    gripperUp();
    delay(1500);
    servo_test.detach();











  */

  dL = UltrasonicLeft();
  dR = UltrasonicRight();

  if (dR > 25.00) {
    dR = UltrasonicRight();
  }
  if (dL > 25.00) {
    dL = UltrasonicLeft();
  }

  if (dL > 50) {
    dL = 0;
  }
  if (dR > 50) {
    dR = 0;
  }

  startTime = millis();
  timerOffset = 10000;
  while (1) { //this is a redundancy. maybe we can remove?

    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dL > 20.00 || dR > 20.00) {
      allMotorStop();
      break;
    }

    goStraight_Backwards(millis());

    if (dL > 50) {
      dL = 0;
    }
    if (dR = 50) {
      dR = 0;
    }

    if (dR > 25.00) {
      dR = UltrasonicRight();
    }
    if (dL > 25.00) {
      dL = UltrasonicLeft();
    }
  }


  burstFwd();
  delay(150);
  allMotorStop();

  Reset_Gyro();
  delay(500);
  turnRightBackwards90();
  allMotorStop();
  delay(500);
  burstBkwd();
  delay(1000);
  allMotorStop();

  int recovCnt = 0;

recovery:
  recovCnt++;

  if (recovCnt > 2) {
    goto endrecovery;
  }

  //dL = UltrasonicLeft();
  dR = UltrasonicRight();



  if (dR > 25.00) {
    //dR = UltrasonicRight();
  }
  if (dL > 25.00) {
    dL = UltrasonicLeft();
  }

  startTime = millis();

  int myCounter = 0;
  bool gap1 = false;
  bool gap2 = false;
  bool RecoveryCount = false;

  while (1) { //this is a redundancy. maybe we can remove?
    if (millis() > (startTime + 20000)) {
      allMotorStop();
      break;
    }
    goStraight_Backwards_L(millis());



    //dL = UltrasonicLeft();

    if (dL > 20.0) {
      RecoveryCount = true;
      //myCounter++;
    }

    if ((dL < 10) && (RecoveryCount == true)) {
      myCounter++;

    }
    RecoveryCount = false;
    //dR = UltrasonicRight();
    /*
      if (dR > 25.00) {
      dR = UltrasonicRight();
      }
      if (dL > 25.00) {
      dL = UltrasonicLeft();
      }
    */
  }

  allMotorStop();
  delay(500);

  /*
    if (myCounter < 1) {
      burstFwd();
      delay(500);
      allMotorStop();
      goto recovery;
    }
  */

endrecovery:


  burstFwd();
  delay(3000);


  allMotorStop();
  delay(200);

  servo_test.attach(11);
  gripperDown();




  delay(1500);



  servo_test.detach();



  burstBkwd();
  delay(6000);

  burstFwd();
  delay(350);
  allMotorStop();

  servo_test.attach(11);

  gripperUp();
  delay(3000);
  servo_test.detach();


  burstBkwd();

  delay(1000);

  allMotorStop();
  delay(100);

  turnRightBackwards90();

  allMotorStop();

  delay(500);








dR = UltrasonicRight();



  if (dR > 25.00) {
    //dR = UltrasonicRight();
  }
  if (dL > 25.00) {
    dL = UltrasonicLeft();
  }

  startTime = millis();


while (1) { //this is a redundancy. maybe we can remove?
    if (millis() > (startTime + 2500)) {
      allMotorStop();
      break;
    }
    goStraight_Backwards_L(millis());
  }

  allMotorStop();
  delay(500);  







  dL = UltrasonicLeft();
  dR = UltrasonicRight();

  if (dR > 25.00) {
    dR = UltrasonicRight();
  }
  if (dL > 25.00) {
    dL = UltrasonicLeft();
  }

  while (dL < 20.0 && dR < 20.0) { //this is a redundancy. maybe we can remove?
    goStraight_BackwardsDownShort(millis());

    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
      dR = UltrasonicRight();
    }
    if (dL > 25.00) {
      dL = UltrasonicLeft();
    }
  }

  allMotorStop();
  delay(500);


  Reset_Gyro();
  turnRightBackwards90();

  burstBkwd();

  delay(3000);

  allMotorStop();

  delay(200);


  dL = UltrasonicLeft();
  dR = UltrasonicRight();

  if (dR > 25.00) {
    dR = UltrasonicRight();
  }
  if (dL > 25.00) {
    dL = UltrasonicLeft();
  }

  while (dL < 20.0 && dR < 20.0) { //this is a redundancy. maybe we can remove?
    goStraight_BackwardsDownShort(millis());

    dL = UltrasonicLeft();
    dR = UltrasonicRight();

    if (dR > 25.00) {
      dR = UltrasonicRight();
    }
    if (dL > 25.00) {
      dL = UltrasonicLeft();
    }
  }

  allMotorStop();
  delay(500);
  


  /*

    Reset_Gyro();

    turnLeftBackwards90();

    allMotorStop();


  */



  while (1) {
    allMotorStop();
  }


}




//Functions








void burstBkwd_Fast() {
  double PWM_R = 1.0 * (255);
  double PWM_L = 1.0 * (255);

  digitalWrite(MotorPinL, CW);// set direction
  analogWrite(MotorSpeedPinL, PWM_L);// set speed at maximum

  digitalWrite(MotorPinR, CW);// set direction
  analogWrite(MotorSpeedPinR, PWM_R);// set speed at maximum
}

void burstFwd() {
  double PWM_R = 0.85 * (100);
  double PWM_L = 1.0 * (100);

  digitalWrite(MotorPinL, CCW);// set direction
  analogWrite(MotorSpeedPinL, PWM_L);// set speed at maximum

  digitalWrite(MotorPinR, CCW);// set direction
  analogWrite(MotorSpeedPinR, PWM_R);// set speed at maximum
}

void burstFwd_R() {
  double PWM_R = 0.65 * (100);
  double PWM_L = 1.0 * (100);

  digitalWrite(MotorPinL, CCW);// set direction
  analogWrite(MotorSpeedPinL, PWM_L);// set speed at maximum

  digitalWrite(MotorPinR, CCW);// set direction
  analogWrite(MotorSpeedPinR, PWM_R);// set speed at maximum
}

void burstBkwd() {
  double PWM_R = 0.825 * (100);
  double PWM_L = 1.0 * (100);

  digitalWrite(MotorPinL, CW);// set direction
  analogWrite(MotorSpeedPinL, PWM_L);// set speed at maximum

  digitalWrite(MotorPinR, CW);// set direction
  analogWrite(MotorSpeedPinR, PWM_R);// set speed at maximum
}

void burstFwd_L() {
  double PWM_R = 0.65 * (100);
  double PWM_L = 1.0 * (100);

  digitalWrite(MotorPinL, CCW);// set direction
  analogWrite(MotorSpeedPinL, PWM_L);// set speed at maximum

  digitalWrite(MotorPinR, CCW);// set direction
  analogWrite(MotorSpeedPinR, PWM_R);// set speed at maximum
}


void burstFwd_MDF() {
  double PWM_R = 0.85 * (100);
  double PWM_L = 1.0 * (100);

  digitalWrite(MotorPinL, CCW);// set direction
  analogWrite(MotorSpeedPinL, PWM_L);// set speed at maximum

  digitalWrite(MotorPinR, CCW);// set direction
  analogWrite(MotorSpeedPinR, PWM_R);// set speed at maximum
}




/***********************************************-go straight *************************************************************************************************/

void goStraight(long startTime) {
  double distR, distL;

  distR = getDistanceRight();
  distL = getDistanceLeft();
  Serial.println("point 1");
  if (distR > 17.00) {
    distR = UltrasonicRight();
    Serial.println("Switching to Ultrasonic Right");
  }
  Serial.println("point 2");
  if (distL > 17.00) {
    distL = UltrasonicLeft();
    Serial.println("Switching to Ultrasonic Left");
  }

  if (distR > 20.0 || distL > 20.0) { //this is a redundancy. maybe we can remove?
    allMotorStop();
    Serial.println("Stopping motors");
    return;
  }

  delay(10);

  error = distR - distL;

  Serial.print("Distance L: "); Serial.print(distL); Serial.print("    Distance R: "); Serial.print(distR); Serial.print("    ERROR: "); Serial.println(error);



  long newTime = millis();

  long elapsedTime = newTime - startTime;

  iError += error * elapsedTime;
  dError = (error - prevError) / (elapsedTime);

  double totalError = (kp * error) + (ki * iError) + (kd * dError);

  PWMR = 0.55 * (100 - totalError);
  PWML = (100 + totalError) ;

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

  //Serial.print ("PWML: "); Serial.print (PWML); Serial.print ("      PWMR: "); Serial.print (PWMR);
  digitalWrite(MotorPinL, CCW);// set direction
  analogWrite(MotorSpeedPinL, PWML);// set speed at maximum

  digitalWrite(MotorPinR, CCW);// set direction
  analogWrite(MotorSpeedPinR, PWMR);// set speed at maximum

  lastPWM_R = PWMR;
  lastPWM_L = PWML;

}

/***********************************************-go straight ramp down Short*************************************************************************************************/

void goStraight_RampDownShort(long startTime) {

  double distR, distL;

  distR = getDistanceRight();
  distL = getDistanceLeft();

  if (distR > 25.00) {
    distR = UltrasonicRight();
  }
  if (distL > 25.00) {
    distL = UltrasonicLeft();
  }

  if (distR > 20.0 || distL > 20.0) { //this is a redundancy. maybe we can remove?
    allMotorStop();
    return;
  }

  delay(10);

  error = distR - distL;

  long newTime = millis();

  long elapsedTime = newTime - startTime;

  iError += error * elapsedTime;
  dError = (error - prevError) / (elapsedTime);
  //kp = 6.5 kd = 13 speed  = 0.4
  double kp_rd = 4; double ki_rd = 0.0; double kd_rd = 1.7;

  double totalError = (kp_rd * error) + (ki_rd * iError) + (kd_rd * dError);

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

  double PWML_Adjusted = (PWML * 0.4);
  double PWMR_Adjusted = (PWMR * 0.4) * 1.30;

  digitalWrite(MotorPinL, CCW);// set direction
  analogWrite(MotorSpeedPinL, PWML_Adjusted);// set speed at maximum

  digitalWrite(MotorPinR, CCW);// set direction
  analogWrite(MotorSpeedPinR, PWMR_Adjusted);// set speed at maximum

  lastPWM_R = PWMR;
  lastPWM_L = PWML;
}




/***********************************************-go straight ramp down Long*************************************************************************************************/

void goStraight_RampDownLong(long startTime) {
  double distR, distL;

  distR = getDistanceRight();
  distL = getDistanceLeft();

  if (distR > 25.00) {
    distR = UltrasonicRight();
  }
  if (distL > 25.00) {
    distL = UltrasonicLeft();
  }

  if (distR > 35.0 || distL > 35.0) { //this is a redundancy. maybe we can remove?
    allMotorStop();
    return;
  }

  delay(10);

  error = distR - distL;

  long newTime = millis();

  long elapsedTime = newTime - startTime;

  iError += error * elapsedTime;
  dError = (error - prevError) / (elapsedTime);
  //kp = 9 kd = 13 speed  = 0.37
  double kp_rd = 9; double ki_rd = 0.0; double kd_rd = 14;

  double totalError = (kp_rd * error) + (ki_rd * iError) + (kd_rd * dError);

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

  double PWML_Adjusted = (PWML * 0.37);
  double PWMR_Adjusted = (PWMR * 0.37);

  digitalWrite(MotorPinL, CCW);// set direction
  analogWrite(MotorSpeedPinL, PWML_Adjusted);// set speed at maximum

  digitalWrite(MotorPinR, CCW);// set direction
  analogWrite(MotorSpeedPinR, PWMR_Adjusted);// set speed at maximum

  lastPWM_R = PWMR;
  lastPWM_L = PWML;
}


/***********************************************-go straight ramp up long*************************************************************************************************/

void goStraight_RampUpLong(long startTime) {
  if ((UltrasonicRight() > 20.0) || (UltrasonicLeft() > 20.0)) { //this is a redundancy. maybe we can remove?
    allMotorStop();
    return;
  }

  delay(10);

  error = getDistanceRight() - getDistanceLeft();

  long newTime = millis();

  long elapsedTime = newTime - startTime;

  iError += error * elapsedTime;
  dError = (error - prevError) / (elapsedTime);
  //kp = 9 kd = 13 speed  = 0.4
  double kp_rd = 5.5; double ki_rd = 0.0; double kd_rd = 3.5;

  double totalError = (kp_rd * error) + (ki_rd * iError) + (kd_rd * dError);

  PWMR = ((100 - totalError) * 2.4) * 0.50 ;
  PWML = (100 + totalError ) * 2.4 ;

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

  double PWML_Adjusted = (PWML);
  double PWMR_Adjusted = (PWMR);

  //Serial.print("PWML: "); Serial.print(PWML); Serial.print("    PWMR: "); Serial.println(PWMR);

  digitalWrite(MotorPinL, CCW);// set direction
  analogWrite(MotorSpeedPinL, PWML_Adjusted);// set speed at maximum

  digitalWrite(MotorPinR, CCW);// set direction
  analogWrite(MotorSpeedPinR, PWMR_Adjusted);// set speed at maximum

  lastPWM_R = PWMR;
  lastPWM_L = PWML;
}




double getDistanceLeft() {
  String msg;
  digitalWrite(IRL, HIGH);
  double distanceL = analogRead(IRLRead);
  distanceL = (distanceL * -0.1499) + 69.793 ;
  if (distanceL == 69.793) {
    distanceL = 0;
  }

  delay(100);
  return distanceL;
}



double getDistanceRight() {
  String msg;
  digitalWrite(IRR, HIGH);
  double distanceR = analogRead(IRRRead);
  distanceR = (distanceR * -0.1499) + 69.793 ;
  if (distanceR == 69.793) {
    distanceR = 0;
  }
  delay(100);
  return distanceR;
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
  double yawTarget = yawInit + 85;


  /////////////////////////// PULL FORWARD SLIGHTLY //////////////////////////

  burstFwd();
  delay(600);


  /////////////////////////// PULL BACK SLIGHTLY /////////////////////////////
  digitalWrite(MotorPinL, CW);// set direction
  analogWrite(MotorSpeedPinL, 45);// set speed
  digitalWrite(MotorPinR, CW);// set direction
  analogWrite(MotorSpeedPinR, 180);// set speed
  delay(400);

  long theStart = millis();
  long theEnd = theStart + 3000;

  while (1) {
    double theVal = getYawDeg();
    Serial.print(theVal); Serial.print("     "); Serial.println(yawTarget);
    if (theVal > yawTarget) {
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

void turnLeft270() {
  double yawInit = getYawDeg();
  double yawTarget = yawInit - 90;


  /////////////////////////// PULL FORWARD SLIGHTLY //////////////////////////

  burstFwd();
  delay(1150);


  /////////////////////////// PULL BACK SLIGHTLY /////////////////////////////
  digitalWrite(MotorPinL, CW);// set direction
  analogWrite(MotorSpeedPinL, 150);// set speed
  digitalWrite(MotorPinR, CW);// set direction
  analogWrite(MotorSpeedPinR, 50);// set speed
  delay(750);

  while (1) {
    double theVal = getYawDeg();
    Serial.print(theVal); Serial.print("     "); Serial.println(yawTarget);
    if (theVal < yawTarget) {
      allMotorStop();
      delay(500);

      /*

        digitalWrite(MotorPinL, CW);// set direction
        analogWrite(MotorSpeedPinL, 95);// set speed

        digitalWrite(MotorPinR, CW);// set direction
        analogWrite(MotorSpeedPinR, 145);// set speed


        delay(1250);

      */

      allMotorStop();

      return;
    }
    else {
      digitalWrite(MotorPinL, CCW);// set direction
      analogWrite(MotorSpeedPinL, 160);// set speed
      digitalWrite(MotorPinR, CCW);// set direction
      analogWrite(MotorSpeedPinR, 10);// set speed
    }
  }
}


void turnLeftBackwards90() {
  double yawInit = getYawDeg();
  double yawTarget = yawInit + 82.0;

  long newStart = millis();

  while (1) {
    double theVal = getYawDeg();
    //Serial.print(theVal); Serial.print("     "); Serial.println(yawTarget);
    Serial.println(millis() - newStart);
    if (millis() > (newStart + 5250)) {
      allMotorStop();
      break;
    }



    if (theVal > yawTarget) {
      allMotorStop();
      //delay(500);

      //burstFwd_MDF();

      delay(650);

      allMotorStop();

      return;

    }
    else {
      digitalWrite(MotorPinL, CW);// set direction
      analogWrite(MotorSpeedPinL, 145);// set speed
      digitalWrite(MotorPinR, CCW);// set direction
      analogWrite(MotorSpeedPinR, 10);// set speed
    }
  }
}


void turnRightBackwards90() {
  double yawInit = getYawDeg();
  double yawTarget = yawInit - 70.0;

  long newStart = millis();

  while (1) {
    double theVal = getYawDeg();
    //Serial.print(theVal); Serial.print("     "); Serial.println(yawTarget);
    Serial.println(millis() - newStart);
    if (millis() > (newStart + 5250)) {
      allMotorStop();
      break;
    }



    if (theVal < yawTarget) {
      allMotorStop();
      //delay(500);

      //burstFwd_MDF();

      delay(650);

      allMotorStop();

      return;

    }
    else {
      digitalWrite(MotorPinL, CCW);// set direction
      analogWrite(MotorSpeedPinL, 10);// set speed
      digitalWrite(MotorPinR, CW);// set direction
      analogWrite(MotorSpeedPinR, 145);// set speed
    }
  }
}



void turnRight90() {

  /////////////////////////// PULL FORWARD SLIGHTLY //////////////////////////

  burstFwd_MDF();
  delay(100);


  /////////////////////////// PULL BACK SLIGHTLY /////////////////////////////
  digitalWrite(MotorPinL, CW);// set direction
  analogWrite(MotorSpeedPinL, 180);// set speed
  digitalWrite(MotorPinR, CW);// set direction
  analogWrite(MotorSpeedPinR, 25);// set speed
  delay(500);
  //Serial.print(getYawDeg());
  double yawInit = getYawDeg();
  double yawTarget = yawInit - 115;

  while (1) {
    double theVal = getYawDeg();
    Serial.print(theVal); Serial.print("     "); Serial.println(yawTarget);
    if (theVal < yawTarget) {
      allMotorStop();
      delay(500);

      burstFwd_MDF();

      delay(650);

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

void turnRight90_MidRamp() {

  /////////////////////////// PULL FORWARD SLIGHTLY //////////////////////////

  burstFwd_MDF();
  delay(100);


  /////////////////////////// PULL BACK SLIGHTLY /////////////////////////////
  digitalWrite(MotorPinL, CW);// set direction
  analogWrite(MotorSpeedPinL, 180);// set speed
  digitalWrite(MotorPinR, CW);// set direction
  analogWrite(MotorSpeedPinR, 25);// set speed
  delay(500);
  //Serial.print(getYawDeg());
  double yawInit = getYawDeg();
  double yawTarget = yawInit - 110;

  while (1) {
    double theVal = getYawDeg();
    Serial.print(theVal); Serial.print("     "); Serial.println(yawTarget);
    if (theVal < yawTarget) {
      allMotorStop();
      delay(500);

      burstFwd_MDF();

      delay(650);

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

void turnRight90_Acrylic() {

  /////////////////////////// PULL FORWARD SLIGHTLY //////////////////////////

  burstFwd();
  delay(500);


  /////////////////////////// PULL BACK SLIGHTLY /////////////////////////////
  digitalWrite(MotorPinL, CW);// set direction
  analogWrite(MotorSpeedPinL, 180);// set speed
  digitalWrite(MotorPinR, CW);// set direction
  analogWrite(MotorSpeedPinR, 25);// set speed
  delay(500);
  //Serial.print(getYawDeg());
  double yawInit = getYawDeg();
  double yawTarget = yawInit - 100;

  while (1) {
    double theVal = getYawDeg();
    Serial.print(theVal); Serial.print("     "); Serial.println(yawTarget);
    if (theVal < yawTarget) {
      allMotorStop();
      delay(500);

      digitalWrite(MotorPinL, CCW);// set direction
      analogWrite(MotorSpeedPinL, 95);// set speed

      digitalWrite(MotorPinR, CCW);// set direction
      analogWrite(MotorSpeedPinR, 190);// set speed

      delay(750);

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



double UltrasonicLeft() {
  // Clears the trigpinL
  digitalWrite(trigpinL, LOW);
  delayMicroseconds(2);
  // Sets the trigpinL on HIGH state for 10 micro seconds
  digitalWrite(trigpinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpinL, LOW);
  // Reads the echopinL, returns the sound wave travel time in microseconds
  double durL = pulseIn(echopinL, HIGH);
  // Calculating the distanceL
  double distL = durL * 0.0343 / 2;
  //Serial.print("Ultrasonic L: "); Serial.print(distanceL);
  if (distL > 50) {
    //distL = 0;
  }
  return distL;
}


double UltrasonicRight() {
  digitalWrite(trigpinR, LOW);
  delayMicroseconds(2);
  // Sets the trigpinR on HIGH state for 10 micro seconds
  digitalWrite(trigpinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpinR, LOW);
  // Reads the echopinR, returns the sound wave travel time in microseconds
  double durR = pulseIn(echopinR, HIGH);
  // Calculating the distanceR
  double distR = durR * 0.0343 / 2;
  //Serial.print("    Ultrasonic R: "); Serial.println(distanceR);
  if (distR > 50) {
    //distR = 0;
  }
  return distR;
}

/***********************************************-ramp down*************************************************************************************************/

void gripperUp() {
  int angle = 50;
  for (angle = 50; angle < 180; angle += 1)
  {
    Serial.println(angle);
    servo_test.write(angle);                 //command to rotate the servo to the specified angle
    //delay(45);
    //delay(3);

  }
  Serial.print("gripperDown");
}
/***********************************************-ramp down*************************************************************************************************/
void gripperDown() {
  int angle = 180;
  for (angle = 180; angle > 50; angle -= 1)
  {
    servo_test.write(angle);                 //command to rotate the servo to the specified angle
    //delay(15);
  }
}



void goStraight_R(long startTime) {
  double distR, distL;

  distR = getDistanceRight();

  Serial.println("point 1");
  if (distR > 25.00) {
    distR = UltrasonicRight();
    Serial.println("Switching to Ultrasonic Right");
  }
  Serial.println("point 2");


  if (distR > 30.0 ) { //this is a redundancy. maybe we can remove?
    allMotorStop();
    Serial.println("Stopping motors");
    return;
  }
  distL = 6.00;

  delay(10);

  error = distR - distL;

  Serial.print("Distance L: "); Serial.print(distL); Serial.print("    Distance R: "); Serial.print(distR); Serial.print("    ERROR: "); Serial.println(error);

  double kp_R = 3.5;
  double ki_R = 0;
  double kd_R = 4.8;

  long newTime = millis();

  long elapsedTime = newTime - startTime;

  iError += error * elapsedTime;
  dError = (error - prevError) / (elapsedTime);

  double totalError = (kp_R * error) + (ki_R * iError) + (kd_R * dError);

  PWMR = 0.74 * (100 - totalError);
  PWML = (100 + totalError) ;

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

  //Serial.print ("PWML: "); Serial.print (PWML); Serial.print ("      PWMR: "); Serial.print (PWMR);
  digitalWrite(MotorPinL, CCW);// set direction
  analogWrite(MotorSpeedPinL, PWML);// set speed at maximum

  digitalWrite(MotorPinR, CCW);// set direction
  analogWrite(MotorSpeedPinR, PWMR);// set speed at maximum

  lastPWM_R = PWMR;
  lastPWM_L = PWML;

}


void goStraight_Backwards(long startTime) {
  double distR, distL;

  double kpBack, kdBack, kiBack;
  //kp = 1, kd = 0
  kpBack = 1.5; kiBack = 0; kdBack = 0;

  distR = UltrasonicRight();
  distL = UltrasonicLeft();
  Serial.print("L: "); Serial.print(distL); Serial.print("    R: "); Serial.println(distR);

  if (distR > 25.00) {
    distR = UltrasonicRight();

  }

  if (distL > 25.00) {
    distL = UltrasonicLeft();

  }

  if (distR > 30.0 || distL > 30.0) { //this is a redundancy. maybe we can remove?
    allMotorStop();

    return;
  }

  delay(10);

  error = distR - distL;

  //Serial.print("Distance L: "); Serial.print(distL); Serial.print("    Distance R: "); Serial.print(distR); Serial.print("    ERROR: "); Serial.println(error);



  long newTime = millis();

  long elapsedTime = newTime - startTime;

  iError += error * elapsedTime;
  dError = (error - prevError) / (elapsedTime);

  double totalError = (kpBack * error) + (kiBack * iError) + (kdBack * dError);

  PWMR = 0.8 * (100 - totalError);
  PWML = (100 + totalError) ;

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

  //Serial.print ("PWML: "); Serial.print (PWML); Serial.print ("      PWMR: "); Serial.print (PWMR);
  digitalWrite(MotorPinL, CW);// set direction
  analogWrite(MotorSpeedPinL, PWML);// set speed at maximum

  digitalWrite(MotorPinR, CW);// set direction
  analogWrite(MotorSpeedPinR, PWMR);// set speed at maximum

  lastPWM_R = PWMR;
  lastPWM_L = PWML;

}







void goStraight_Backwards_R(long startTime) {
  double distR, distL;

  double kpBack, kdBack, kiBack;
  //kp = 1, kd = 0
  kpBack = 1.5; kiBack = 0; kdBack = 0;

  distR = UltrasonicRight();
  distL = UltrasonicLeft();
  Serial.print("L: "); Serial.print(distL); Serial.print("    R: "); Serial.println(distR);

  if (distR > 25.00) {
    distR = UltrasonicRight();

  }

  if (distL > 25.00) {
    distL = UltrasonicLeft();

  }

  if (distR > 30.0 || distL > 30.0) { //this is a redundancy. maybe we can remove?
    allMotorStop();

    return;
  }

  delay(10);

  error = distL - 6.0;

  //Serial.print("Distance L: "); Serial.print(distL); Serial.print("    Distance R: "); Serial.print(distR); Serial.print("    ERROR: "); Serial.println(error);



  long newTime = millis();

  long elapsedTime = newTime - startTime;

  iError += error * elapsedTime;
  dError = (error - prevError) / (elapsedTime);

  double totalError = (kpBack * error) + (kiBack * iError) + (kdBack * dError);

  PWMR = 0.8 * (100 - totalError);
  PWML = (100 + totalError) ;

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

  //Serial.print ("PWML: "); Serial.print (PWML); Serial.print ("      PWMR: "); Serial.print (PWMR);
  digitalWrite(MotorPinL, CW);// set direction
  analogWrite(MotorSpeedPinL, PWML);// set speed at maximum

  digitalWrite(MotorPinR, CW);// set direction
  analogWrite(MotorSpeedPinR, PWMR);// set speed at maximum

  lastPWM_R = PWMR;
  lastPWM_L = PWML;

}





void goStraight_Backwards_L(long startTime) {
  double distR, distL;

  double kpBack, kdBack, kiBack;
  //kp = 1, kd = 0
  kpBack = 2.0; kiBack = 0; kdBack = 2;

  distR = UltrasonicRight();
  distL = UltrasonicLeft();
  Serial.print("L: "); Serial.print(distL); Serial.print("    R: "); Serial.println(distR);

  if (distR > 25.00) {
    distR = UltrasonicRight();

  }

  if (distL > 25.00) {
    distL = UltrasonicLeft();

  }

  if (distR > 30.0 || distL > 30.0) { //this is a redundancy. maybe we can remove?
    allMotorStop();

    return;
  }

  delay(10);

  error = distR - 2.5;

  //Serial.print("Distance L: "); Serial.print(distL); Serial.print("    Distance R: "); Serial.print(distR); Serial.print("    ERROR: "); Serial.println(error);



  long newTime = millis();

  long elapsedTime = newTime - startTime;

  iError += error * elapsedTime;
  dError = (error - prevError) / (elapsedTime);

  double totalError = (kpBack * error) + (kiBack * iError) + (kdBack * dError);

  PWMR = 0.8 * (100 - totalError);
  PWML = (100 + totalError) ;

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

  //Serial.print ("PWML: "); Serial.print (PWML); Serial.print ("      PWMR: "); Serial.print (PWMR);
  digitalWrite(MotorPinL, CW);// set direction
  analogWrite(MotorSpeedPinL, PWML);// set speed at maximum

  digitalWrite(MotorPinR, CW);// set direction
  analogWrite(MotorSpeedPinR, PWMR);// set speed at maximum

  lastPWM_R = PWMR;
  lastPWM_L = PWML;

}







void goStraight_BackwardsDownShort(long startTime) {
  double distR, distL;

  double kpBack, kdBack, kiBack;
  //kp = 1, kd = 0
  kpBack = 3; kiBack = 0; kdBack = 0;

  distR = getDistanceRight();
  distL = getDistanceLeft();

  if (distR > 25.00) {
    distR = UltrasonicRight();

  }

  if (distL > 25.00) {
    distL = UltrasonicLeft();

  }

  //Serial.print("L: "); Serial.print(distL); Serial.print("    R: "); Serial.println(distR);

  if (distR > 30.0 || distL > 30.0) { //this is a redundancy. maybe we can remove?
    allMotorStop();

    return;
  }

  delay(10);

  error = distR - distL;

  //Serial.print("Distance L: "); Serial.print(distL); Serial.print("    Distance R: "); Serial.print(distR); Serial.print("    ERROR: "); Serial.println(error);



  long newTime = millis();

  long elapsedTime = newTime - startTime;

  iError += error * elapsedTime;
  dError = (error - prevError) / (elapsedTime);

  double totalError = (kpBack * error) + (kiBack * iError) + (kdBack * dError);

  PWMR = 0.50 * (100 - totalError);
  PWML = (100 + totalError) ;

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

  //Serial.print ("PWML: "); Serial.print (PWML); Serial.print ("      PWMR: "); Serial.print (PWMR);
  digitalWrite(MotorPinL, CW);// set direction
  analogWrite(MotorSpeedPinL, PWML);// set speed at maximum

  digitalWrite(MotorPinR, CW);// set direction
  analogWrite(MotorSpeedPinR, PWMR);// set speed at maximum

  lastPWM_R = PWMR;
  lastPWM_L = PWML;

}






void goStraight_BackwardsDownLong(long startTime) {
  double distR, distL;

  double kpBack, kdBack, kiBack;
  //kp = 3, kd = 0
  kpBack = 1.5; kiBack = 0; kdBack = 0;

  distR = getDistanceRight();
  distL = getDistanceLeft();

  if (distR > 25.00) {
    distR = UltrasonicRight();

  }

  if (distL > 25.00) {
    distL = UltrasonicLeft();

  }

  //Serial.print("L: "); Serial.print(distL); Serial.print("    R: "); Serial.println(distR);

  if (distR > 30.0 || distL > 30.0) { //this is a redundancy. maybe we can remove?
    allMotorStop();

    return;
  }

  delay(10);

  error = distR - distL;

  //Serial.print("Distance L: "); Serial.print(distL); Serial.print("    Distance R: "); Serial.print(distR); Serial.print("    ERROR: "); Serial.println(error);



  long newTime = millis();

  long elapsedTime = newTime - startTime;

  iError += error * elapsedTime;
  dError = (error - prevError) / (elapsedTime);

  double totalError = (kpBack * error) + (kiBack * iError) + (kdBack * dError);

  PWMR = 0.50 * (100 - totalError);
  PWML = (100 + totalError) ;

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

  //Serial.print ("PWML: "); Serial.print (PWML); Serial.print ("      PWMR: "); Serial.print (PWMR);
  digitalWrite(MotorPinL, CW);// set direction
  analogWrite(MotorSpeedPinL, PWML);// set speed at maximum

  digitalWrite(MotorPinR, CW);// set direction
  analogWrite(MotorSpeedPinR, PWMR);// set speed at maximum

  lastPWM_R = PWMR;
  lastPWM_L = PWML;

}





















void goUp_BackwardsLong(long startTime) {
  double distR, distL;

  double kpBack, kdBack, kiBack;
  // 1.3    1.2
  kpBack = 1.3; kiBack = 0; kdBack = 1.3;

  distR = UltrasonicRight();
  distL = UltrasonicLeft();
  //Serial.print("L: "); Serial.print(distL); Serial.print("    R: "); Serial.println(distR);

  if (distR > 25.00) {
    distR = UltrasonicRight();

  }

  if (distL > 25.00) {
    distL = UltrasonicLeft();

  }

  if (distR > 30.0 || distL > 30.0) { //this is a redundancy. maybe we can remove?
    allMotorStop();

    return;
  }

  delay(10);

  error = distR - distL;

  //Serial.print("Distance L: "); Serial.print(distL); Serial.print("    Distance R: "); Serial.print(distR); Serial.print("    ERROR: "); Serial.println(error);



  long newTime = millis();

  long elapsedTime = newTime - startTime;

  iError += error * elapsedTime;
  dError = (error - prevError) / (elapsedTime);

  double totalError = (kpBack * error) + (kiBack * iError) + (kdBack * dError);

  PWMR = ((100 - totalError) * 2.4) * 0.78;
  PWML = (100 + totalError) * 2.4 ;

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

  //Serial.print ("PWML: "); Serial.print (PWML); Serial.print ("      PWMR: "); Serial.print (PWMR);
  digitalWrite(MotorPinL, CW);// set direction
  analogWrite(MotorSpeedPinL, PWML);// set speed at maximum

  digitalWrite(MotorPinR, CW);// set direction
  analogWrite(MotorSpeedPinR, PWMR);// set speed at maximum

  lastPWM_R = PWMR;
  lastPWM_L = PWML;

}
/*
   double kp_rd = 5.5; double ki_rd = 0.0; double kd_rd = 3.5;

  double totalError = (kp_rd * error) + (ki_rd * iError) + (kd_rd * dError);

  PWMR = ((100 - totalError) * 2.4) * 0.50 ;
  PWML = (100 + totalError ) * 2.4 ;
*/
