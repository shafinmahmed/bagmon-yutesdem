//Use ultrasonics to move correct motors to move at the same speed

/********************----Motor pin decleration ---*******************************************************************************/
const int MotorPinR = 7; // for motor A
const int MotorSpeedPinR = 6; // for motor A


const int MotorPinL = 4; // for motor B
const int MotorSpeedPinL = 5;// for motor B

const int CW  = HIGH;
const int CCW = LOW;
/******************************************************************************************************************************/

/**********************---- PID Vars ---------*********************************************************************************/

long oldTime;
long newTime;
const double kp = 50.0; //Change to tune
const double ki = 0.0; //Change to tune
const double kd = 0.0; //Change to tune
double error, iError, dError, prevError;


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

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  pinMode(trigpinL, OUTPUT);
  pinMode(trigpinR, OUTPUT);
  pinMode(MotorPinR, OUTPUT);
  pinMode(MotorPinL, OUTPUT);
  pinMode(MotorSpeedPinR, OUTPUT);
  pinMode(MotorSpeedPinL, OUTPUT);

  pinMode(echopinR, INPUT);
  pinMode(echopinL, INPUT);

  digitalWrite(MotorPinR, CCW);
  digitalWrite(MotorPinL, CCW);

  analogWrite(MotorSpeedPinR, 255);
  analogWrite(MotorSpeedPinL, 255);

  oldTime = millis();

}

void loop() {
  // put your main code here, to run repeatedly:
  double distL = calculateSideDistance(true);
  double distR = calculateSideDistance(false);
  double PWM_L, PWM_R;
  newTime = millis();
  if (newTime - oldTime > 100) {
    double PID_val = computePID(newTime, oldTime);
    if (distL - distR > 0) { //need to move to right
      PWM_R = 255 - PID_val;
      analogWrite(MotorSpeedPinR, PWM_R);
      Serial.print("PWM_R: ");
      Serial.println(PWM_R);
    }
    if (distR - distL > 0) { //need to move to left
      PWM_L = 255 - PID_val;
      analogWrite(MotorSpeedPinL, PWM_L);
      Serial.print("PWM_L: ");
      Serial.println(PWM_L);
    }
    oldTime = newTime;
  }
}

double calculateSideDistance(bool left) {
  // ++++++ Getting data for ultrasonics ++++++

  // Clear trig pins
  digitalWrite(trigpinL, LOW);
  digitalWrite(trigpinR, LOW);
  
  delayMicroseconds(2);

  // Set trigs to HIGH
  digitalWrite(trigpinL, HIGH);
  digitalWrite(trigpinR, HIGH);

  // Soundwave travel time
  durationL = pulseIn(echopinL, HIGH);
  durationR = pulseIn(echopinR, HIGH);

  // Calculate sound wave travel distance
  distanceL = durationL * 0.034 / 2;
  distanceR = durationR * 0.034 / 2;

  if (left == true) {
    return distanceL;
  }
  else {
    return distanceR;
  }
}

double computePID (long currTime, long prevTime) {
  double distL = calculateSideDistance(true);
  double distR = calculateSideDistance(false);
  
  long elapsedTime = (currTime - prevTime);
  error = distL - distR;
  iError += error * (elapsedTime/ 1000);
  dError = (error - prevError) / (elapsedTime / 1000);

  prevError = error;

  double totalError = (kp * error) + (ki * iError) + (kd * dError);

  double returnVal = map(abs(totalError), 0, 170, 0, 255);
}
