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
    String ins = mySerial.readStringUntil('\n'); 
    ins.trim();
    Serial.println(ins);
    int sep1 = ins.indexOf(':');
    int sep2 = ins.indexOf(':', sep1 + 1);
    
    if (sep1 != -1 && sep2 != -1) {
      String cmd = ins.substring(0, sep1);   
      cmd.trim();        
      int Output1 = ins.substring(sep1 + 1, sep2).toInt(); 
      int Output2 = ins.substring(sep2 + 1).toInt();

    
    // if (cmd=="A" || cmd=="G" || cmd=="D") {
      avancer(HIGH,HIGH,Output1,Output2);
    // }
    // else {
    //   avancer(LOW,LOW,0,0);
    // }
    }
  }
}
 

void avancer(char G, char D, int vg, int vd) {
  digitalWrite(M1, G);  // moteur gauche
  digitalWrite(M2, D);
  analogWrite(E1, vg);
  analogWrite(E2, vd);
} 