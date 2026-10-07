//#include "SoftwareSerial.h"
//#include "Ultrasonic.h"
//SoftwareSerial mySerial(1,0); // broshes TX, RX
//Ultrasonic ultrasonic(13);
//int distance;

#define E1 5
#define M1 4
#define E2 6
#define M2 2
#define Enc1 7
#define Enc2 8
#define BP A0

bool CAPG, CAPC, CAPD;
int val = 1;
int rpm1, rpm2;
volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
unsigned long lastTime = 0;

void setup() {
  //mySerial.begin(9600);
  Serial.begin(9600);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(BP, INPUT);
  pinMode(Enc1, INPUT);
  pinMode(Enc2, INPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(Enc1), countPulses1, RISING);
  attachInterrupt(digitalPinToInterrupt(Enc2), countPulses2, RISING);
}

void loop() {
  //  if (digitalRead(BP)==HIGH) {
  //    val=0;}
  //if (val==1){
  //line();
  //    avancer(HIGH,HIGH,155,0);
  //    vitesse();
  //  }
  //  else {
  //    avancer(LOW, LOW, 0,0);
  //  }
}

void countPulses1() {
  pulseCount1++;
}

void countPulses2() {
  pulseCount2++;
}

void vitesse() {
  if (millis() - lastTime >= 1000) {
    rpm1 = (pulseCount1 / 20.0) * 60.0;
    rpm2 = (pulseCount2 / 20.0) * 60.0;
    Serial.print(rpm1);
    Serial.print(" , ");
    Serial.println(rpm2);
    pulseCount1 = 0;
    pulseCount2 = 0;
    lastTime = millis();
  }
}

void line() {
  CAPG = digitalRead(8);
  CAPC = digitalRead(9);
  CAPD = digitalRead(10);
  if (CAPC) {
    avancer(HIGH, HIGH, 155, 155);
  } else if (CAPG) {
    avancer(HIGH, HIGH, 100, 155);
  } else if (CAPD) {
    avancer(HIGH, HIGH, 155, 100);
  } else {
    //ult();
    avancer(LOW, LOW, 0, 0);
  }
}

void avancer(char G, char D, int vg, int vd) {
  digitalWrite(M1, G);  // moteur gauche
  digitalWrite(M2, D);
  analogWrite(E1, vg);
  analogWrite(E2, vd);
}

//void app {
//  if (mySerial.available()) {
//    val = mySerial.read();
//    Serial.print(val);
//    if (val == '1'){
//      {line();}  }
//    else if (val == '0')
//      {avancer(LOW,0,0);} }
//  if (digitalRead(BP) == HIGH)
//    {avancer(LOW,0,0);}}

//void ult() {
//long distance;
//  distance = ultrasonic.read();
//  delay(50);
//  Serial.print(distance);
//  Serial.println("cm");
//  if(distance == 7){
//    avancer(HIGH,155,155);}
//  if(distance < 5) {
//    avancer(HIGH,155,100);}
//  if(distance > 9){
//    avancer(HIGH,100,155);}}