#include "SoftwareSerial.h" 
//#include "Ultrasonic.h"
SoftwareSerial mySerial(11,10); // broches TX, RX
SoftwareSerial mySerialBT(12,13); // broches TX, RX
//Ultrasonic ultrasonic(13);
//int distance;
#include <PID_v1.h>

#define BP A0
#define Enc1 2
#define Enc2 3


bool CAPG, CAPC, CAPD;

volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
unsigned long lastTime = 0;
int rpm1=0, rpm2=0;

int m1=0,m2=0,m3=0,mAg=0,mGg=0,mDg=0,mAd=0,mGd=0,mDd=0;
bool affmoy=false;
bool actif=false;

String Cmd;

int rpm100; //à trouver si fonction proportionnel
double Setpoint1=155, Input1, Output1, Setpoint2=155, Input2, Output2;
double Kp=2,Ki=5,Kd=1;
PID myPID1(&Input1, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint2, Kp, Ki, Kd, DIRECT);

void setup() {
  mySerial.begin(9600);
  mySerialBT.begin(9600);
  Serial.begin(9600);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(BP, INPUT);
  pinMode(Enc1, INPUT);
  pinMode(Enc2, INPUT);
  attachInterrupt(digitalPinToInterrupt(Enc1), countPulses1, RISING);
  attachInterrupt(digitalPinToInterrupt(Enc2), countPulses2, RISING);
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
}

void loop() {
  app();      
  //  if (digitalRead(BP)==HIGH) {
  //    val=0;}
  //if (val==1){
  //    line();
  //    avancer(HIGH,HIGH,155,0);
  //    vitesse();
  //  }
  //  else {
  //    avancer(LOW, LOW, 0,0);
  //  }
}

void calculPWM() {
  Input1 = 100*rpm1/rpm100; // trouver PWM fonction de rpm
  Input2 = 100 *rpm2/rpm100;

  myPID1.Compute();
  myPID2.Compute();
  Output1 = constrain(Output1, 0, 255);
  Output2 = constrain(Output2, 0, 255);
}

void countPulses1() {
  pulseCount1++;
}

void countPulses2() {
  pulseCount2++;
}

void vitesse() {
  if (millis() - lastTime >= 100) {
    rpm1 = (pulseCount1 / 20.0) * 600.0;
    rpm2 = (pulseCount2 / 20.0) * 600.0;
    Serial.print(rpm1);
    Serial.print(" , ");
    Serial.println(rpm2);
    pulseCount1 = 0;
    pulseCount2 = 0;
    lastTime = millis();
    moyenne();
  }
}

void moyenne() {
  if (rpm1!=0 || rpm2!=0 || Cmd=="S") {
    if (Cmd=="A"){
    m1++;
    mAd+=rpm1;
    mAg+=rpm2;
    affmoy=false;
  }
  else if (Cmd=="G"){
    m2++;
    mGd+=rpm1;
    mGg+=rpm2;
    affmoy=false;
  }
  else if (Cmd=="D"){
    m3++;
    mDd+=rpm1;
    mDg+=rpm2;
    affmoy=false;
  }
  else if (!affmoy && (m1!=0 && m2!=0 && m3!=0)) {
    //Serial.print("vitesse moyenne en ligne droite des moteurs gauche : ");
    //Serial.print(mAg/m1);
    //Serial.print(" rpm sur ");
    //Serial.print(m1);
    //Serial.print(" valeurs, des moteurs droit : ");
    //Serial.print(mAd/m1);
    //Serial.print(" rpm sur ");
    //Serial.print(m1);
    //Serial.println(" valeurs");
    //Serial.print("vitesse moyenne en tournant gauche des moteurs gauche : ");
    //Serial.print(mGg/m2);
    //Serial.print(" rpm sur ");
    //Serial.print(m2);
    //Serial.print(" valeurs, des moteurs droit : ");
    //Serial.print(mGd/m2);
    //Serial.print(" rpm sur ");
    //Serial.print(m2);
    //Serial.println(" valeurs");
    //Serial.print("vitesse moyenne en tournant droit des moteurs gauche : ");
    //Serial.print(mDg/m3);
    //Serial.print(" rpm sur ");
    //Serial.print(m3);
    //Serial.print(" valeurs, des moteurs droit : ");
    //Serial.print(mDd/m3);
    //Serial.print(" rpm sur ");
    //Serial.print(m3);
    //Serial.println(" valeurs");

    mySerialBT.print(mAg/m1);
    mySerialBT.print(",");
    mySerialBT.print(mAd/m1);
    mySerialBT.print(",");
    mySerialBT.print(mGg/m2);
    mySerialBT.print(",");
    mySerialBT.print(mGd/m2);
    mySerialBT.print(",");
    mySerialBT.print(mDg/m3);
    mySerialBT.print(",");
    mySerialBT.println(mDd/m3);
    
    affmoy=true;
  }
  }
}

void line() {
  CAPG = digitalRead(4);
  CAPC = digitalRead(5);
  CAPD = digitalRead(6);

  if (CAPC) {
    Setpoint1 = 155;
    Setpoint2 = 155;
    Cmd="A";
  } 
  else if (CAPG) {
    Setpoint1 = 100;
    Setpoint2 = 155;
    Cmd="G";
  } 
  else if (CAPD) {
    Setpoint1 = 155;
    Setpoint2 = 100;
    Cmd="D";
  } 
  else {
    //ult();
    Cmd="S";
  }

  mySerial.print(Cmd);
  mySerial.print(":");
  mySerial.print(Output1);
  mySerial.print(":");
  mySerial.println(Output2);
}


void app() {
  if (mySerialBT.available()) {
    String val = mySerialBT.readStringUntil('\n');
    val.trim();
  //  Serial.println(val);
  if (val == "1") {
      actif = true;
    } 
    else if (val == "0") {
      actif = false;
      mySerial.println("S");
    } 
    else if (val == "moyenne") {
      affmoy = false;
    }
  }

  if (digitalRead(BP) == HIGH) {
    actif = false;
    mySerial.println("S");
  }

  if (actif) {
    calculPWM();
    line();
    vitesse();
  }
}

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