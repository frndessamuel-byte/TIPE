#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <Ultrasonic.h>

SoftwareSerial mySerial(12, 13); // broshes TX, RX
#define E1 5
#define E2 6
#define M1 4
#define M2 7
#define BP A0
#define Enc1 1
#define Enc2 10
bool CAPG, CAPC, CAPD;

Ultrasonic ultrasonic(2);
int val;

volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
unsigned long lastTime = 0;
int rpm100=100; //tour par min pour E1=100
int rpm1,rpm2;

double Setpoint1=155, Input1, Output1, Setpoint2=155, Input2, Output2;
double Kp=2, Ki=5, Kd=1;
double dt=100,previousError1=0,previousError2,integral1=0,integral2=0;

void setup() {
  mySerial.begin(9600);
  pinMode(3, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(BP, INPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(Enc1), countPulses1, RISING);
  attachInterrupt(digitalPinToInterrupt(Enc2), countPulses2, RISING);
}

void loop() {
  Input1 = 100*rpm1/rpm100;
  Input2 = 100*rpm2/rpm100;
  pid();
  Output1 = constrain(Output1, 0, 255);
  Output2 = constrain(Output2, 0, 255);
  app();
} 
void pid() {
  if (millis() - lastTime >= 100) {
    int error1 = Setpoint1 - Input1;
    int error2 = Setpoint2 - Input2;
    integral1 += error1 * dt;
    integral2 += error2 * dt;
    int derivative1 = (error1 - previousError1) / dt;
    int derivative2 = (error2 - previousError2) / dt;
    Output1 = Kp * error1 + Ki * integral1 + Kd * derivative1;
    Output2 = Kp * error2 + Ki * integral2 + Kd * derivative2;
    previousError1 = error1;
    previousError2 = error2;
  }
}
void app() {
  if (mySerial.available()) {

    val = mySerial.read();
    Serial.print(val);
    if (val == '1'){
      {line();}  }
    else if (val == '0') 
      {arret();}
  }

  if (digitalRead(BP) == HIGH)
    {arret();}
}


void countPulses1() {
  pulseCount1++;
}

void countPulses2() {
  pulseCount2++;
}

void vitesse(){
  if (millis() - lastTime >= dt) {
    float rpm1 = (pulseCount1 / 20.0) * 600.0;
    float rpm2 = (pulseCount2 / 20.0) * 600.0;
    Serial.print(rpm1);
    Serial.print(",");
    Serial.println(rpm2);
    pulseCount1 = 0;
    pulseCount2 = 0;
    lastTime = millis();
  }
}

void line() {
  CAPG = digitalRead(3);
  CAPC = digitalRead(8);
  CAPD = digitalRead(9);
  if (CAPC) {
    avancer();}
  if (CAPG) {
    gauche(); }
  if (CAPD) {
    droite(); }
  if (!CAPG && !CAPC && ! CAPD) {
    ult();
  }
}

void ult() {
long distance;
  distance = ultrasonic.read();
  delay(50);
  Serial.print(distance);
  Serial.println("cm");

  if(distance == 7){
    avancer();
} 
  if(distance < 5) {
    droite();
}
  if(distance > 9){
    gauche();
}
}

void avancer() {
  digitalWrite(M1,HIGH);
  digitalWrite(M2,HIGH);
  analogWrite(E1,(int)Output1);
  analogWrite(E2,(int)Output2);
  
}
void gauche() {
  digitalWrite(M1,HIGH);
  digitalWrite(M2,HIGH);
  analogWrite(E1,(int)Output1);
  analogWrite(E2,(int)Output2-55);
  
}
void droite() { 
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1,(int)Output1-55);
  analogWrite(E2,(int)Output2);
}
void arret() {
    digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  analogWrite(E1,0);
  analogWrite(E2,0);
}