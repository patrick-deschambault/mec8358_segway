#include <I2Cdev.h>

#define  BufferSize   11            //Input Buffer to incoming comunications
String   InputString = "";          // a string to hold incoming data
boolean  StringComplete = false;    // Flag, whether the string is complete
unsigned long speed;
int inputVoltage = 0;
int inputDirection = 1;

const int motorPin = 10;  // Pin pour contrôler la vitesse du moteur
const int dirPin1 = 7;  // Pin pour la direction 1A
const int dirPin2 = 8;  // Pin pour la direction 2A
const int encoderPin = 2;  // Pin de l'encodeur optique
volatile int encoderCount = 0;  // Compteur d'encoches
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;  // Pour calculer la vitesse
unsigned long Start_time = 0;  // Pour plot la vitesse
unsigned long interval = 50;  // Intervalle pour mesurer la vitesse
float Motor_speed = 0;  // Vitesse du moteur (en tours/minute)
float stepsPerRevolution = 240;

const int voltage_max  = 10;

void setup() {
  Serial.begin(115200);
  
  pinMode(motorPin, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(encoderPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countEncoder, RISING);  // Détection du front montant
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
  analogWrite(motorPin, speed);  // PWM pour contrôler la vitesse du moteur

   // Calculer la vitesse du moteur chaque seconde
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Motor_speed = (encoderCount / stepsPerRevolution) * (60000.0/interval);  // inserer le nombre de steps par tour ici 
    Serial.print((currentMillis - Start_time)/1000.0);
    Serial.print(",");
    Serial.println(Motor_speed);
    encoderCount = 0;  // Réinitialiser le compteur d'encoches
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
void countEncoder() {
  encoderCount++;
}