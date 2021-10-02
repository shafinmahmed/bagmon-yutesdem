const int MotorPinA = 4; // for motor A
const int MotorSpeedPinA = 5; // for motor A
//const int MotorBrakePinA = 9; // for motor A


const int MotorPinB = 7; // for motor B
const int MotorSpeedPinB = 6;// for motor B
//const int MotorBrakePinB = 8;// for motor B

const int CW  = HIGH;
const int CCW = LOW;



//a comment


void setup() {
  // motor A pin assignment
  pinMode(MotorPinA, OUTPUT);
  pinMode(MotorSpeedPinA, OUTPUT);
  //pinMode(MotorBrakePinA, OUTPUT);

  // motor B pin assignment
  pinMode(MotorPinB, OUTPUT);
  pinMode(MotorSpeedPinB, OUTPUT);
  //pinMode(MotorBrakePinB, OUTPUT); 


  Serial.begin(9600);//  seial monitor initialized 

}



void loop() {

 digitalWrite(MotorPinB, CW);// set direction
      Serial.println("Direction CW"); 
  analogWrite(MotorSpeedPinB, 100);// set speed at maximum

 digitalWrite(MotorPinA, CW);// set direction
      Serial.println("Direction CW"); 
  analogWrite(MotorSpeedPinA, 100);// set speed at maximum


}// loop end
