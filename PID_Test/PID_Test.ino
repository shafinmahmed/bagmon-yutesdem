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
const int trigpinR = 11;
const int echopinR = 12;
// defines variables
long durationR;
int distanceR;
/******************************************************************************************************************************/



/********************----Ultrasonic R pin decleration ---*******************************************************************************/
// defines pins numbers
const int trigpinL = 9;
const int echopinL = 10;
// defines variables
long durationL;
int distanceL;
/******************************************************************************************************************************/




/********************----PID Constants decleration ---*******************************************************************************/
int Ultrasonic_Error = 0;

/*
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
*/
int maxError = 0;
double prevError = 0.0;
double error = 0.0;
double iError = 0.0;
double dError = 0.0;

double kp = 15;   double ki = 0;   double kd = 5; //Adjust as needed

int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;
int PID_values_fixed =0;

int PWMR = 255;
int PWML = 255;

long oldTime, newTime;

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

  oldTime = millis();

  digitalWrite(MotorPinL, CCW);// set direction
  analogWrite(MotorSpeedPinL, 255);// set speed at maximum
  
  digitalWrite(MotorPinR, CCW);// set direction
  analogWrite(MotorSpeedPinR, 255);// set speed at maximum

}

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
  //Serial.print(distanceL);
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
  //Serial.print("     distanceR: ");
  //Serial.println(distanceR);
  /*********************************************************************************************************************************/

  
  /********************----Calculate PID Value ---*******************************************************************************/

  delay(10);

  error = distanceL - distanceR;
 
  newTime = millis();

  long elapsedTime = newTime - oldTime;

  oldTime = newTime;

  iError += error * (elapsedTime);
  dError = (error - prevError) / (elapsedTime);

  prevError = error;

  double totalError = ((kp * error) + (ki * iError) + (kd * dError));


  Serial.print("TOTAL ERROR: ");
  Serial.print(totalError);


  /*if (abs(totalError) > maxError){
    maxError = abs(totalError);
  }*/


  double PWM_adjustment = map(abs(totalError), 0, 100, 0, 255);

  if (abs(totalError) > 100) {
    PWM_adjustment = 255;
  }

  Serial.print("     PWM ADJUSTMENT: ");
  Serial.print(PWM_adjustment);

  if (totalError > 0) { //has to adjust left
    analogWrite(MotorSpeedPinL, (255 - PWM_adjustment));
    analogWrite(MotorSpeedPinR, 255);
    
   
    Serial.print(" PWMR: ");
    Serial.print(255);
    Serial.print(" PWML: ");
    Serial.println((255 - PWM_adjustment));
    
  }
  else { //has to adjust right
    analogWrite(MotorSpeedPinR, (255 - PWM_adjustment));
    analogWrite(MotorSpeedPinL, 255);
    
    Serial.print(" PWMR: ");
    Serial.print((255 - PWM_adjustment));
    Serial.print(" PWML: ");
    Serial.println(255);
    
  }
}// loop end
