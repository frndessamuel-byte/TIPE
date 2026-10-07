#include "SoftwareSerial.h" 
//#include "Ultrasonic.h"
SoftwareSerial mySerial(11,10); 
SoftwareSerial mySerialBT(12,13); // Bluetooth 
//Ultrasonic ultrasonic(13);
//int distance;
#include <PID_v1.h>

#define BP A0      // Bouton poussoir
#define Enc1 2     // Encodeur moteur 1
#define Enc2 3     // Encodeur moteur 2

// Variables pour capteurs de ligne
bool CAPG, CAPC, CAPD;

// Variables pour compter les impulsions des encodeurs
volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
unsigned long lastTime = 0;
float omega1=0, omega2=0, omega155=12.56; // Vitesses moteurs

// Variables pour calcul de moyennes
int m1=0,m2=0,m3=0,mAg=0,mGg=0,mDg=0,mAd=0,mGd=0,mDd=0;
bool affmoy=false;
bool actif=false;

String Cmd; // Commande courante

// Variables PID
double Setpoint1=155, Input1, Output1, Setpoint2=155, Input2, Output2;
double Kp=2,Ki=6,Kd=0.005116;
PID myPID1(&Input1, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint2, Kp, Ki, Kd, DIRECT);

void setup() {
  // Initialisation des communications série
  mySerial.begin(9600);      
  mySerialBT.begin(9600);    
  Serial.begin(9600);       

  // Initialisation des broches pour les capteurs et encodeurs 
  pinMode(4, INPUT);         // Capteur gauche
  pinMode(5, INPUT);         // Capteur central
  pinMode(6, INPUT);         // Capteur droit
  pinMode(BP, INPUT);        // Bouton poussoir
  pinMode(Enc1, INPUT);      // Encodeur 1
  pinMode(Enc2, INPUT);      // Encodeur 2
  attachInterrupt(digitalPinToInterrupt(Enc1), countPulses1, RISING); // Interruption encodeur 1
  attachInterrupt(digitalPinToInterrupt(Enc2), countPulses2, RISING); // Interruption encodeur 2
  myPID1.SetMode(AUTOMATIC); // PID automatique
  myPID2.SetMode(AUTOMATIC);
}

void loop() {
  app(); // Boucle principale
}

// Calcul des PWM à partir des vitesses mesurées et PID
void calculPWM() {
  Input1 = 155*omega1/omega155; 
  Input2 = 155 *omega2/omega155;

  myPID1.Compute();
  myPID2.Compute();
  Output1 = constrain(Output1, 0, 200); // Limiter PWM
  Output2 = constrain(Output2, 0, 200);
}

// Compteur d'impulsions encodeur 1 
void countPulses1() {
  pulseCount1++;
}

// Compteur d'impulsions encodeur 2
void countPulses2() {
  pulseCount2++;
}

// Calcul de la vitesse des moteurs à partir des impulsions
void vitesse() {
  if (millis() - lastTime >= 100) { // Toutes les 100 ms
    omega1 = (pulseCount1 / 20.0) * 20.0 * 3.14159; // Calcul vitesse moteur 1
    omega2 = (pulseCount2 / 20.0) * 20.0 * 3.14159; // Calcul vitesse moteur 2
    mySerialBT.print(omega1);
    mySerialBT.print(" , ");
    mySerialBT.println(omega2);
    pulseCount1 = 0;
    pulseCount2 = 0;
    lastTime = millis();
    moyenne(); // Calcul des moyennes
  }
}

// Calcul des moyennes des vitesses selon la commande
void moyenne() {
  if (omega1!=0 || omega2!=0 || Cmd=="S") {
    if (Cmd=="A"){
      m1++;
      mAd+=omega1;
      mAg+=omega2;
      affmoy=false;
    }
    else if (Cmd=="G"){ 
      m2++;
      mGd+=omega1;
      mGg+=omega2;
      affmoy=false;
    }
    else if (Cmd=="D"){ 
      m3++;
      mDd+=omega1;
      mDg+=omega2;
      affmoy=false;
    }
    else if (!affmoy && (m1!=0 && m2!=0 && m3!=0)) {
      affmoy=true;
    }
  }
}

// Lecture des capteurs de ligne et choix de la commande
void line() {
  CAPG = digitalRead(4); // Capteur gauche
  CAPC = digitalRead(5); // Capteur central
  CAPD = digitalRead(6); // Capteur droit

  if (CAPC) { 
    Setpoint1 = 100;
    Setpoint2 = 155;
    Cmd="A";
  } 
  else if (CAPG) { 
    Setpoint1 = 155; 
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
    Cmd="S"; // Stop
  }

  // Affichage des commandes et PWM sur le port série
  mySerial.print(Cmd);
  mySerial.print(":");
  mySerial.print(Output1);
  mySerial.print(":");
  mySerial.println(Output2);
}

// Application principale : gestion Bluetooth, bouton, calculs et actions
void app() {
  if (mySerialBT.available()) { // Si message Bluetooth reçu
    String val = mySerialBT.readStringUntil('\n');
    val.trim();
    //Serial.println(val);
    if (val == "1") { // Activation
      actif = true;
    } 
    else if (val == "0") { // Désactivation
      actif = false;
      mySerial.println("S");
    } 
    else if (val == "moyenne") { // Demande des moyennes
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
      affmoy = false;
    }
  }

  if (digitalRead(BP) == HIGH) { // Arrêt d'urgence par bouton
    actif = false;
    mySerial.println("S");
  }

  if (actif) { // Si actif, exécuter les fonctions principales
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