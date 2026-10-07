#include "SoftwareSerial.h"
SoftwareSerial mySerial(10,11); // 
// Définition des broches pour les moteurs
#define E1 5   // PWM moteur gauche
#define M1 4   // Direction moteur gauche
#define E2 6   // PWM moteur droit
#define M2 2   // Direction moteur droit
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
  // Vérifie si des données sont disponibles sur le port série logiciel
  if (mySerial.available()){
    String ins = mySerial.readStringUntil('\n'); // Lit la ligne reçue jusqu'au saut de ligne
    ins.trim(); // Supprime les espaces inutiles
    //Serial.println(ins);
    int sep1 = ins.indexOf(':'); // Cherche le premier séparateur ':'
    int sep2 = ins.indexOf(':', sep1 + 1); // Cherche le second séparateur ':'
    
    // Vérifie que les deux séparateurs existent
    if (sep1 != -1 && sep2 != -1) {
      String cmd = ins.substring(0, sep1);   // Récupère la commande (A, G, D, etc.)
      cmd.trim();        
      int Output1 = ins.substring(sep1 + 1, sep2).toInt(); // Récupère la valeur du moteur gauche
      int Output2 = ins.substring(sep2 + 1).toInt();       // Récupère la valeur du moteur droit

      Serial.println(cmd); // Affiche la commande reçue sur le port série

      // Si la commande est reconnue, fait avancer les moteurs
      if (cmd=="A" || cmd=="G" || cmd=="D") {
        avancer(HIGH,HIGH,Output1,Output2);
      }
      // Sinon, arrête les moteurs
      else {
        avancer(LOW,LOW,0,0);
      }
    }
  }
}
 
// Fonction pour contrôler les moteurs
void avancer(char G, char D, int vg, int vd) {
  digitalWrite(M1, G);  // Définit la direction du moteur gauche
  digitalWrite(M2, D);  // Définit la direction du moteur droit
  analogWrite(E1, vg);  // Définit la vitesse du moteur gauche (PWM)
  analogWrite(E2, vd);  // Définit la vitesse du moteur droit (PWM)
} 