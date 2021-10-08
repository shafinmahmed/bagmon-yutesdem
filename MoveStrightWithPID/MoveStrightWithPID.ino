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




/********************----Ultrasonic L pin decleration ---*******************************************************************************/
// defines pins numbers
const int trigpinR = 9;
const int echopinR = 10;
// defines variables
long durationR;
int distanceR;
/******************************************************************************************************************************/



/********************----Ultrasonic R pin decleration ---*******************************************************************************/
// defines pins numbers
const int trigpinL = 11;
const int echopinL = 12;
// defines variables
long durationL;
int distanceL;
/******************************************************************************************************************************/




/********************----PID Constants decleration ---*******************************************************************************/
int Ultrasonic_Error = 0;

float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;

int kp = 50;   int ki = 15;   int kd = 15; //Adjust as needed

int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;
int PID_values_fixed =0;

int PWMR = 255;
int PWML = 255;





/******************************************************************************************************************************/













/********************----Setup ---*****************************************************************************************************************************************************************/
/********************----Setup ---*****************************************************************************************************************************************************************/
void setup() {

/********************---- ---*******************************************************************************/
/*********************************************************************************************************************/

Serial.begin(9600);//  seial monitor initialized 

   
/********************----Ultrasonic L Setup ---*******************************************************************************/
pinMode(trigpinL, OUTPUT); // Sets the trigpinL as an Output
pinMode(echopinL, INPUT); // Sets the echopinL as an Input
/**********************************************************************************************************************/

/********************----Ultrasonic R Setup ---*******************************************************************************/
pinMode(trigpinR, OUTPUT); // Sets the trigpinR as an Output
pinMode(echopinR, INPUT); // Sets the echopinR as an Input
/**********************************************************************************************************************/



   
/********************----Motor Setup ---*******************************************************************************/
  // motor A pin assignment
  pinMode(MotorPinR, OUTPUT);
  pinMode(MotorSpeedPinR, OUTPUT);
  //pinMode(MotorBrakePinA, OUTPUT);

  // motor B pin assignment
  pinMode(MotorPinL, OUTPUT);
  pinMode(MotorSpeedPinL, OUTPUT);
  //pinMode(MotorBrakePinB, OUTPUT); 
/**********************************************************************************************************************/


 

}









/********************----Loop ---*****************************************************************************************************************************************************************/
/********************----Loop ---*****************************************************************************************************************************************************************/

void loop() {



/********************----Run Ultrasonic L Loop Code ---*******************************************************************************/

// Clears the trigpinL
digitalWrite(trigpinL, LOW);
delayMicroseconds(2);
// Sets the trigpinL on HIGH state for 10 micro seconds
digitalWrite(trigpinL, HIGH);
delayMicroseconds(10);
digitalWrite(trigpinL, LOW);
// Reads the echopinL, returns the sound wave travel time in microseconds
durationL = pulseIn(echopinL, HIGH);
// Calculating the distanceL
distanceL= durationL*0.034/2;
// Prints the distanceL on the Serial Monitor
//Serial.print("distanceL: ");
//Serial.println(distanceL);
/*********************************************************************************************************************************/


/********************----Run Ultrasonic R Loop Code ---*******************************************************************************/

// Clears the trigpinR
digitalWrite(trigpinR, LOW);
delayMicroseconds(2);
// Sets the trigpinR on HIGH state for 10 micro seconds
digitalWrite(trigpinR, HIGH);
delayMicroseconds(10);
digitalWrite(trigpinR, LOW);
// Reads the echopinL, returns the sound wave travel time in microseconds
durationR = pulseIn(echopinR, HIGH);
// Calculating the distanceR
distanceR= durationR*0.034/2;
// Prints the distanceR on the Serial Monitor
//Serial.print("distanceR: ");
//Serial.println(distanceR);
/*********************************************************************************************************************************/





/********************----Calculate PID Value ---*******************************************************************************/

PID_error = distanceR - distanceL;
//Calculate the P value
PID_p = 0.01*kp * PID_error;
//Calculate the I value in a range on +-3
PID_i = 0.01*PID_i + (ki * PID_error);


//For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 
  //Now we can calculate the D calue
  PID_d = 0.01*kd*((PID_error - previous_error)/elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;

//Serial.println(PID_value);
/*
//We define PWM range between 0 and 255
  if(PID_value < -255)
  {    PID_value = 0;    }
  if(PID_value > 255)  
  {    PID_value = 255;  }
*/

PWMR = (100 + PID_value);
PWML = (100 - PID_value);


 previous_error = PID_error;

if (PWMR < 0){
  PWMR = 0;
}
if (PWMR > 255){
  PWMR = 255;
}


if (PWML < 0){
  PWML = 0;
}
if (PWML > 255){
  PWML = 255;
}
Serial.print("PWML: ");
Serial.print(PWML);

Serial.print("           PWMR: ");
Serial.println(PWMR);




/*********************************************************************************************************************************/










/********************----Run Motor Loop Code ---**********************************************************************************/
/*
 digitalWrite(MotorPinB, CW);// set direction
  analogWrite(MotorSpeedPinB, 100);// set speed at maximum
 digitalWrite(MotorPinA, CW);// set direction
  analogWrite(MotorSpeedPinA, 100);// set speed at maximum
  */
/*
if (distanceR > distanceL) {
//Serial.println("distanceR");
  digitalWrite(MotorPinL, CCW);// set direction
  analogWrite(MotorSpeedPinL, 40);// set speed at maximum
 digitalWrite(MotorPinR, CCW);// set direction
  analogWrite(MotorSpeedPinR, 100);// set speed at maximum
  
}
else {
  //Serial.println("distanceL");
  digitalWrite(MotorPinL, CCW);// set direction
  analogWrite(MotorSpeedPinL, 100);// set speed at maximum
 digitalWrite(MotorPinR, CCW);// set direction
  analogWrite(MotorSpeedPinR, 40);// set speed at maximum
}
*/
 digitalWrite(MotorPinL, CCW);// set direction
  analogWrite(MotorSpeedPinL, PWML);// set speed at maximum

 digitalWrite(MotorPinR, CCW);// set direction
  analogWrite(MotorSpeedPinR, PWMR);// set speed at maximum
/**********************************************************************************************************************************/






}// loop end