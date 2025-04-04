#include <I2Cdev.h>

#define  BufferSize   11            //Input Buffer to incoming comunications
String   InputString = "";          // a string to hold incoming data
boolean  StringComplete = false;    // Flag, whether the string is complete
unsigned long speed;
int inputVoltage = 0;
int inputDirection = 1;

const int motorPins[2] = {10, 11};  // Pin pour contrôler la vitesse du moteur

const int dirPin1 = 7;  // Pin pour la direction 1A
const int dirPin2 = 8;  // Pin pour la direction 2A

const int encoderPins[2] = {2, 3};  // Pin de l'encodeur optique
volatile int encoderCount[2] = {0, 0};  // Compteur d'encoches

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;  // Pour calculer la vitesse
unsigned long Start_time = 0;  // Pour plot la vitesse
unsigned long interval = 50;  // Intervalle pour mesurer la vitesse
float Motor_speeds[2] = {0,0};  // Vitesse du moteur (en tours/minute)

float stepsPerRevolution = 240;

const int voltage_max  = 100;


void parseInput();
void countEncoder_0();
void countEncoder_1();

void setup() {
  Serial.begin(115200);
  
  pinMode(motorPins[0], OUTPUT);
  pinMode(motorPins[1], OUTPUT);

  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);

  pinMode(encoderPins[0], INPUT);
  pinMode(encoderPins[1], INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPins[0]), countEncoder_0, RISING);  // Détection du front montant
  attachInterrupt(digitalPinToInterrupt(encoderPins[1]), countEncoder_1, RISING);  // Détection du front montant
}

void loop() {
  if (StringComplete){
    parseInput();
    speed = map(inputVoltage,0,voltage_max,0,255);
    InputString = "";
    StringComplete = false;
    Start_time = currentMillis;
  }
  // Contrôler la direction
  if (inputDirection == 1) {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
  } else if (inputDirection == 2) {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
  } else{
    Serial.println("Direction entrée invalide");
  }
  // Contrôler la vitesse
  analogWrite(motorPins[0], speed);  // PWM pour contrôler la vitesse du moteur
  analogWrite(motorPins[1], speed);  // PWM pour contrôler la vitesse du moteur


   // Calculer la vitesse du moteur chaque seconde
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {

    previousMillis = currentMillis;
    Motor_speeds[0] = (encoderCount[0] / stepsPerRevolution) * (60000.0/interval);  // inserer le nombre de steps par tour ici 
    Motor_speeds[1] = (encoderCount[1] / stepsPerRevolution) * (60000.0/interval);  // inserer le nombre de steps par tour ici 

    Serial.print((currentMillis - Start_time)/1000.0); Serial.print("\t");
    Serial.print(Motor_speeds[0]); Serial.print("\t");
    Serial.println(Motor_speeds[1]);

    encoderCount[0] = 0;  // Réinitialiser le compteur d'encoches
    encoderCount[1] = 0;
  }
}

// Gestion de la communication série et extraction des valeurs
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '.') {
      StringComplete = true;
    } else {
      if (inChar != '\0') InputString += inChar;
    }
    if (InputString.length() >= BufferSize) {
      Serial.println("Buffer Overloaded");
      InputString = "";
    }
  }
}

// Fonction pour analyser l'entrée utilisateur
void parseInput() {
  int commaIndex = InputString.indexOf(',');
  if (commaIndex > 0) {
    // Voltage compris entre 0 et 24V de la source de tension
    inputVoltage = InputString.substring(0, commaIndex).toInt();
    // Direction 1 pour gauche et 2 pour droite
    inputDirection = InputString.substring(commaIndex + 1).toInt();
  } else {
    inputVoltage = InputString.toInt();
  }
}

// Fonction d'interruption pour compter les encoches
void countEncoder_0() {
  encoderCount[0]++;
}

void countEncoder_1() {
  encoderCount[1]++;
}