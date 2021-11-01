 double distance;

void setup() {
  Serial.begin(9600);
  pinMode(2, OUTPUT); //enable pin
  //pinMode(52, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(2, HIGH);
  distance = analogRead(A0);
  distance = (distance * -0.1499) +69.793 ;
  Serial.println(distance);
  
  delay(100);
}
