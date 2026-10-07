#include "SoftwareSerial.h" 
//#include "Ultrasonic.h"
SoftwareSerial mySerial(11,10); // broches TX, RX
SoftwareSerial mySerialBT(12,13); // broches TX, RX
//Ultrasonic ultrasonic(13);
//int distance;

#define BP A0
#define Enc1 2
#define Enc2 3


bool CAPG, CAPC, CAPD;

volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
unsigned long lastTime = 0;
int rpm1, rpm2;

int e1=0,e2=0,e3=0,sAg=0,sGg=0,sDg=0,sAd=0,sGd=0,sDd=0,mAg=0,mGg=0,mDg=0,mAd=0,mGd=0,mDd=0;
bool affmoy=false;
bool actif = false;

String currentCmd;
String lastCmd = "";

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
}

void loop() {
  app();      
  //  if (digitalRead(BP)==HIGH) {
  //    val=0;}
  //if (val==1){
  //    line();
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
  if (millis() - lastTime >= 100) {
    rpm1 = (pulseCount1 / 20.0) * 600.0;
    rpm2 = (pulseCount2 / 20.0) * 600.0;
    //Serial.print(rpm1);
    //Serial.print(" , ");
    //Serial.println(rpm2);
    pulseCount1 = 0;
    pulseCount2 = 0;
    lastTime = millis();
    moyenne();
  }
}

void moyenne() {
  if (rpm1!=0 || rpm2!=0 || currentCmd=="S") {
    if (currentCmd=="A"){
    e1++;
    sAd+=rpm1;
    sAg+=rpm2;
    affmoy=false;
  }
  else if (currentCmd=="G"){
    e2++;
    sGd+=rpm1;
    sGg+=rpm2;
    affmoy=false;
  }
  else if (currentCmd=="D"){
    e3++;
    sDd+=rpm1;
    sDg+=rpm2;
    affmoy=false;
  }
  else if (!affmoy && (e1!=0 && e2!=0 && e3!=0)) {
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
    mAg=sAg/e1;
    mAd=sAd/e1;
    mGg=sGg/e2;
    mGd=sGd/e2;
    mDg=sDg/e3;
    mDd=sDd/e3;
    affmoy=true;
  }
  }
}

void line() {
  CAPG = digitalRead(4);
  CAPC = digitalRead(5);
  CAPD = digitalRead(6);

  if (CAPC) {
    currentCmd="A";
  } 
  else if (CAPG) {
    currentCmd="G";
  } 
  else if (CAPD) {
    currentCmd="D";
  } 
  else {
    //ult();
    currentCmd="S";
  }


  if (currentCmd != lastCmd) {
    mySerial.println(currentCmd);
    Serial.println(currentCmd);
    lastCmd = currentCmd;
  }
}


void app() {
  if (mySerialBT.available()) {
    String val = mySerialBT.readStringUntil('\n');
    val.trim();
    //Serial.println(val);
    //Serial.print(val=="1" );
    //Serial.print(val=="0" );
    //Serial.println(val=="moyenne" );
    if (val == "1") {
      actif = true;
    } 
    else if (val == "0") {
      actif = false;
      mySerial.println("S");
    } 
    else if (val == "moyenne") {
      mySerialBT.print(mAg);
      mySerialBT.print(",");
      mySerialBT.print(mAd);
      mySerialBT.print(",");
      mySerialBT.print(mGg);
      mySerialBT.print(",");
      mySerialBT.print(mGd);
      mySerialBT.print(",");
      mySerialBT.print(mDg);
      mySerialBT.print(",");
      mySerialBT.println(mDd);
      affmoy = false;
    }
  }

  if (digitalRead(BP) == HIGH) {
    actif = false;
    mySerial.println("S");
  }

  if (actif) {
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