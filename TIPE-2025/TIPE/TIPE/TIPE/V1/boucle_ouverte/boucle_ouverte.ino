#include "SoftwareSerial.h"
#include "Ultrasonic.h"

SoftwareSerial mySerial(1,0); // broshes TX, RX
#define E1 4
#define E2 7
#define M1 5
#define M2 6
bool CAPG, CAPC, CAPD;


Ultrasonic ultrasonic(2);
int val;

void setup() {
  mySerial.begin(9600);
  pinMode(3, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
}

void loop() {
  app();
}

void app() {
  if (mySerial.available()) {
    val = mySerial.read();
    Serial.print(val);
    if (val == '1'){
      {avancer();}  }
    else if (val == '0') 
      {arret();}
  }
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
    arret();
  }
}

void avancer() {
  digitalWrite(M1,HIGH);
  digitalWrite(M2,HIGH);
  analogWrite(E1,150);
  analogWrite(E2,150);
}

void gauche() {
  digitalWrite(M1,HIGH);
  digitalWrite(M2,HIGH);
  analogWrite(E1,100);
  analogWrite(E2, 45);
  
}
void droite() { 
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1,45);
  analogWrite(E2,100);
}
void arret() {
    digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  analogWrite(E1,0);
  analogWrite(E2,0);
}
