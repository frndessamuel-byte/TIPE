#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <Ultrasonic.h>

SoftwareSerial mySerial(12, 13); // broshes TX, RX
#define E1 5
#define E2 6
#define M1 4
#define M2 7
#define BP A0
#define PIN_INPUT A1
bool CAPG, CAPC, CAPD;

Ultrasonic ultrasonic(2);
int val;

double Setpoint, Input, Output;
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  mySerial.begin(9600);
  pinMode(3, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(BP, INPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  Input = analogRead(PIN_INPUT);
  Setpoint = 100;
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  Input = analogRead(PIN_INPUT);
  erreur=Setpoint-Input;
  if (erreur>50) {
    Kp=5;
    Ki=10;
    Kd=2;
  }
  else if (erreur>20) {
    Kp=3;
    Ki=7;
    Kd=1.5;
  }
  else {
    Kp=1;
    Ki=3;
    Kd=0.5;
  }
  myPID.SetTunings(Kp,Ki,Kd);
  myPID.Compute();
  Output = constrain(Output, 0, 255);
  app();
} 

void app() {
  if (mySerial.available()) {

    val = mySerial.read();
    Serial.print(val);
    if (val == '1'){
      {line();}  }
    else if (val == '0') 
      {arret();}
  }

  if (digitalRead(BP) == HIGH)
    {arret();}
}

void line() {
  CAPG = digitalRead(3);
  CAPC = digitalRead(8);
  CAPD = digitalRead(9);
  if (CAPC) {
    avancer();}
  if (CAPG) {
    gauche(); }
  if (CAPD) {
    droite(); }
  if (!CAPG && !CAPC && ! CAPD) {
    ult();
  }
}

void ult() {
long distance;
  distance = ultrasonic.read();
  delay(50);
  Serial.print(distance);
  Serial.println("cm");

  if(distance == 7){
    avancer();
} 
  if(distance < 5) {
    droite();
}
  if(distance > 9){
    gauche();
}
}

void avancer() {
  digitalWrite(M1,HIGH);
  digitalWrite(M2,HIGH);
  analogWrite(E1,(int)Output);
  analogWrite(E2,(int)Output);
  
}
void gauche() {
  digitalWrite(M1,HIGH);
  digitalWrite(M2,HIGH);
  analogWrite(E1,(int)Output);
  analogWrite(E2,(int)Output-55);
  
}
void droite() { 
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1,(int)Output-55);
  analogWrite(E2,(int)Output);
}
void arret() {
    digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  analogWrite(E1,0);
  analogWrite(E2,0);
}