//#include "SoftwareSerial.h"
//#include "Ultrasonic.h"
//SoftwareSerial mySerial(1,0); // broshes TX, RX
//Ultrasonic ultrasonic(13);
//int distance;

#include <PID_v1.h>

#define E1 4
#define E2 7
#define M1 5
#define M2 6
#define Enc1 2
#define Enc2 3
#define BP A0

bool CAPG, CAPC, CAPD;
int val=1;
int rpm1,rpm2;
volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
unsigned long lastTime = 0;

int rpm100=100; //à trouver si fonction proportionnel

double Setpoint1=155, Input1, Output1, Setpoint2=155, Input2, Output2;
double Kp=2, Ki=5, Kd=1;
PID myPID1(&Input1, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint2, Kp, Ki, Kd, DIRECT);


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
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
}

void loop() {
  Input1 = 100*rpm1/rpm100; // trouver PWM fonction de rpm
  Input2 = 100 *rpm2/rpm100;
  myPID1.Compute();
  myPID2.Compute();
  Output1 = constrain(Output1, 0, 255);
  Output2 = constrain(Output2, 0, 255);
  if (digitalRead(BP)==HIGH) {
    val=0;}
  if (val==1){
    //line();
    avancer(HIGH,155,155);
    vitesse();}
  else {
    avancer(LOW,0,0);}
} 

void countPulses1() {
  pulseCount1++;
}

void countPulses2() {
  pulseCount2++;
}

void vitesse(){
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
    Setpoint1 = 155;
    Setpoint2 = 155;
    avancer(HIGH,Output1,Output2);}
  else if (CAPG) {
    Setpoint1 = 100; //valeur cible pour tourner à gauche
    Setpoint2 = 155;
    avancer(HIGH,Output1,Output2);} //tourner à gauche
  else if (CAPD) {
    Setpoint1 = 155;
    Setpoint2 = 100; //valeur cible pour tourner à droite
    avancer(HIGH,Output1,Output2);}
  else {
    avancer(LOW,0,0);}
}

void avancer(char L,int vg,int vd) {
  digitalWrite(M1,L);
  digitalWrite(M2,L);
  analogWrite(E1,vg);
  analogWrite(E2,vd);
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