#include "SoftwareSerial.h"
SoftwareSerial mySerial(10,11); // broshes TX, RX

#define E1 5
#define M1 4
#define E2 6
#define M2 2


void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
}

void loop() {
  if (mySerial.available()){
    String cmd=mySerial.readStringUntil('\n');
    cmd.trim();
    Serial.println(cmd);
    if (cmd=="A") {
      avancer(HIGH,HIGH,155,155);
    }
    else if (cmd=="G") {
      avancer(HIGH,HIGH,100,155);
    }
    else if (cmd=="D") {
      avancer(HIGH,HIGH,155,100);
    }
    else {
      avancer(LOW,LOW,0,0);
    }
  }
}
 

void avancer(char G, char D, int vg, int vd) {
  digitalWrite(M1, G);  // moteur gauche
  digitalWrite(M2, D);
  analogWrite(E1, vg);
  analogWrite(E2, vd);
} 