#include "SoftwareSerial.h"
SoftwareSerial mySerial(10, 11); // Création d'un objet pour la communication sur les broches 10 et 11
// Définition des broches utilisées pour le contrôle des moteurs
#define E1 5    // Broche de contrôle (PWM) pour la vitesse du moteur gauche
#define M1 4    // Broche de direction pour le moteur gauche
#define E2 6    // Broche de contrôle (PWM) pour la vitesse du moteur droit
#define M2 2    // Broche de direction pour le moteur droit
void setup() {
  // Initialisation de la communication série pour le SoftwareSerial et le port série principal
  mySerial.begin(9600);
  Serial.begin(9600);
  
  // Définition des broches des moteurs en mode sortie
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
}
void loop() {
  // Vérification des données disponibles sur la liaison série logicielle
  if (mySerial.available()){
  // Lecture de la commande reçue jusqu'au caractère '\n' de passage à la ligne
  String cmd = mySerial.readStringUntil('\n');
  cmd.trim(); // Supprime les espaces superflus en début et fin de chaîne
  // Exemple de traitement des commandes reçues:
  if (cmd == "A") {
      avancer(HIGH, HIGH, 155, 155); // Faire avancer les moteurs à une vitesse standard
  }
  else if (cmd == "G") {
  avancer(HIGH, HIGH, 100, 155); // Tourner à gauche: réduire la vitesse du moteur gauche}
  }
  else if (cmd == "D") {
  avancer(HIGH, HIGH, 155, 100); // Tourner à droite: réduire la vitesse du moteur droit}
  }
  else {
  avancer(LOW, LOW, 0, 0); // Arrêter les moteurs si la commande est non reconnue
  }
  }
}


// Fonction pour contrôler l'avancée des moteurs
void avancer(char G, char D, int vg, int vd) {
  digitalWrite(M1, G);  // Appliquer l'état de direction sur le moteur gauche
  digitalWrite(M2, D);  // Appliquer l'état de direction sur le moteur droit
  analogWrite(E1, vg);  // Régler la vitesse du moteur gauche
  analogWrite(E2, vd);  // Régler la vitesse du moteur droit
} 