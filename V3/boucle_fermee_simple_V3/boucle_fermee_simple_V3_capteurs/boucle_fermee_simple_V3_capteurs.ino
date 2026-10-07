#include "SoftwareSerial.h" 
//#include "Ultrasonic.h"

// Déclaration des ports série logiciels pour communication
SoftwareSerial mySerial(11,10); 
SoftwareSerial mySerialBT(12,13); //Bluetooth
//Ultrasonic ultrasonic(13);
//int distance;
 
#define BP A0      // Bouton poussoir
#define Enc1 2     // Encodeur roue 1
#define Enc2 3     // Encodeur roue 2

// Variables pour capteurs de ligne
bool CAPG, CAPC, CAPD;

// Variables pour comptage des impulsions des encodeurs
volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
unsigned long lastTime = 0;
float omega1=0, omega2=0, omega155=12.56;

// Variables pour calculs de moyennes
int m1=0,m2=0,m3=0,mAg=0,mGg=0,mDg=0,mAd=0,mGd=0,mDd=0;
bool affmoy=false;
bool actif=false;

String Cmd;

// Paramètres PID simplifié
int Kp=10; // Coefficient de proportionnalité pour le PID
double Setpoint1=155, Input1, Output1, Setpoint2=155, Input2, Output2;

void setup() {
  // Initialisation des communications série
  mySerial.begin(9600);      
  mySerialBT.begin(9600);    
  Serial.begin(9600);       

  // Initialisation des broches pour les capteurs et encodeurs 
  pinMode(4, INPUT);         // Capteur ligne gauche
  pinMode(5, INPUT);         // Capteur ligne centre
  pinMode(6, INPUT);         // Capteur ligne droite
  pinMode(BP, INPUT);        // Bouton poussoir
  pinMode(Enc1, INPUT);      // Encodeur 1
  pinMode(Enc2, INPUT);      // Encodeur 2
  attachInterrupt(digitalPinToInterrupt(Enc1), countPulses1, RISING); // Interruption encodeur 1
  attachInterrupt(digitalPinToInterrupt(Enc2), countPulses2, RISING); // Interruption encodeur 2
}

void loop() {
  app();      // Boucle principale
}

// Calcul du PWM à appliquer selon la vitesse mesurée et la consigne
void calculPWM() {
  Input1 = 155*omega1/omega155; // trouver PWM fonction de rpm
  Input2 = 155 *omega2/omega155;

  int erreur1=Setpoint1-Input1;
    Output1=Kp*erreur1;
  int erreur2=Setpoint2-Input2;
    Output2=Kp*erreur2;

  Output1 = constrain(Output1, 0, 255); // Limite la sortie PWM
  Output2 = constrain(Output2, 0, 255);
}

// Fonction d'interruption pour compter les impulsions de l'encodeur 1
void countPulses1() {
  pulseCount1++;
}

// Fonction d'interruption pour compter les impulsions de l'encodeur 2
void countPulses2() {
  pulseCount2++;
}

// Calcul de la vitesse des roues à partir des impulsions des encodeurs
void vitesse() {
  if (millis() - lastTime >= 100) { // Toutes les 100 ms
    omega1 = (pulseCount1 / 20.0) * 600.0; // Calcul du RPM roue 1
    omega2 = (pulseCount2 / 20.0) * 600.0; // Calcul du RPM roue 2
    mySerialBT.print(omega1);
    mySerialBT.print(" , ");
    mySerialBT.println(omega2);
    pulseCount1 = 0;
    pulseCount2 = 0;
    lastTime = millis();
    moyenne(); // Mise à jour des moyennes
  }
}

// Calcul des moyennes de vitesse pour chaque commande
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

// Lecture des capteurs de ligne et choix de la commande à envoyer
void line() {
  CAPG = digitalRead(4); // Capteur gauche
  CAPC = digitalRead(5); // Capteur centre
  CAPD = digitalRead(6); // Capteur droite

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
    //ult(); // Ultrason désactivé
    Cmd="S"; // Stop
  }

  // Envoi de la commande et des PWM sur le port série
  mySerial.print(Cmd);
  mySerial.print(":");
  mySerial.print(Output1);
  mySerial.print(":");
  mySerial.println(Output2);
}

// Fonction principale de gestion de l'application
void app() {
  if (mySerialBT.available()) { // Si une commande Bluetooth est reçue
    String val = mySerialBT.readStringUntil('\n');
    val.trim();
  //  Serial.println(val);
  if (val == "1") {
      actif = true; // Activation du robot
    } 
    else if (val == "0") {
      actif = false; // Désactivation du robot
      mySerial.println("S");
    } 
    else if (val == "moyenne") { // Demande d'affichage des moyennes
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

  if (digitalRead(BP) == HIGH) { // Arrêt d'urgence par bouton poussoir
    actif = false;
    mySerial.println("S");
  }

  if (actif) { // Si le robot est actif
    calculPWM(); // Calcul du PWM à appliquer
    line();      // Lecture des capteurs de ligne et envoi de la commande
    vitesse();   // Calcul de la vitesse des roues
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