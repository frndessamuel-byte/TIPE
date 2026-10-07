#include "SoftwareSerial.h" 
//#include "Ultrasonic.h"

// Déclaration des ports série logiciels pour communication
SoftwareSerial mySerial(11,10); 
SoftwareSerial mySerialBT(12,13); // Bluetooth
//Ultrasonic ultrasonic(13);
//int distance;

// Définition des broches pour les capteurs et boutons
#define BP A0 
#define Enc1 2
#define Enc2 3

// Variables pour les capteurs de ligne
bool CAPG, CAPC, CAPD;

// Variables pour compter les impulsions des encodeurs
volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
unsigned long lastTime = 0;
float omega1, omega2;

// Variables pour calcul des moyennes et statistiques
int e1=0,e2=0,e3=0,sAg=0,sGg=0,sDg=0,sAd=0,sGd=0,sDd=0,mAg=0,mGg=0,mDg=0,mAd=0,mGd=0,mDd=0;
bool affmoy=false;
bool actif = false;

// Variables pour la gestion des commandes
String currentCmd;
String lastCmd = "";

// Initialisation des ports et interruptions
void setup() {
  mySerial.begin(9600);
  mySerialBT.begin(9600);
  Serial.begin(9600);
  pinMode(4, INPUT);    // Capteur gauche
  pinMode(5, INPUT);    // Capteur central
  pinMode(6, INPUT);    // Capteur droit
  pinMode(BP, INPUT);   // Bouton poussoir
  pinMode(Enc1, INPUT); // Encodeur 1
  pinMode(Enc2, INPUT); // Encodeur 2
  attachInterrupt(digitalPinToInterrupt(Enc1), countPulses1, RISING); // Interruption sur encodeur 1
  attachInterrupt(digitalPinToInterrupt(Enc2), countPulses2, RISING); // Interruption sur encodeur 2
}

// Boucle principale
void loop() {
  app();      
}

// Fonction d'interruption pour l'encodeur 1
void countPulses1() {
  pulseCount1++;
}

// Fonction d'interruption pour l'encodeur 2
void countPulses2() {
  pulseCount2++;
}

// Calcul de la vitesse des roues à intervalles réguliers
void vitesse() {
  if (millis() - lastTime >= 100) { // toutes les 100 ms
    omega1 = (pulseCount1 / 20.0) * 20.0 * 3.14159; // Calcul vitesse roue 1
    omega2 = (pulseCount2 / 20.0) * 20.0 * 3.14159; // Calcul vitesse roue 2
    mySerialBT.print(omega1);
    mySerialBT.print(" , ");
    mySerialBT.println(omega2);
    pulseCount1 = 0;
    pulseCount2 = 0;
    lastTime = millis();
    moyenne(); // Mise à jour des moyennes
  }
}

// Calcul des moyennes pour chaque commande
void moyenne() {
  if (omega1!=0 || omega2!=0 || currentCmd=="S") {
    if (currentCmd=="A"){ // Avancer
      e1++;
      sAd+=omega1;
      sAg+=omega2;
      affmoy=false;
    }
    else if (currentCmd=="G"){ // Tourner à gauche
      e2++;
      sGd+=omega1;
      sGg+=omega2;
      affmoy=false;
    }
    else if (currentCmd=="D"){ // Tourner à droite
      e3++;
      sDd+=omega1;
      sDg+=omega2;
      affmoy=false;
    }
    else if (!affmoy && (e1!=0 && e2!=0 && e3!=0)) { // Calcul des moyennes si toutes les directions ont été testées
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

// Lecture des capteurs de ligne et détermination de la commande à envoyer
void line() {
  CAPG = digitalRead(4); // Capteur gauche
  CAPC = digitalRead(5); // Capteur central
  CAPD = digitalRead(6); // Capteur droit

  if (CAPC) {
    currentCmd="A"; // Avancer
  } 
  else if (CAPG) {
    currentCmd="G"; // Tourner à gauche
  } 
  else if (CAPD) {
    currentCmd="D"; // Tourner à droite
  } 
  else {
    //ult();
    currentCmd="S"; // Stop si aucun capteur n'est activé
  }

  // Envoi de la commande si elle a changé
  if (currentCmd != lastCmd) {
    mySerial.println(currentCmd);
    Serial.println(currentCmd);
    lastCmd = currentCmd;
  }
}

// Fonction principale de gestion de l'application
void app() {
  // Lecture des commandes reçues par Bluetooth
  if (mySerialBT.available()) {
    String val = mySerialBT.readStringUntil('\n');
    val.trim();
    if (val == "1") {
      actif = true; // Activation du suivi de ligne
    } 
    else if (val == "0") {
      actif = false; // Désactivation
      mySerial.println("S");
    } 
    else if (val == "moyenne") { // Demande d'affichage des moyennes
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

  // Arrêt d'urgence via bouton poussoir
  if (digitalRead(BP) == HIGH) {
    actif = false;
    mySerial.println("S");
  }

  // Si le robot est actif, suivi de ligne et calcul de vitesse
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