#include <I2Cdev.h>

#define  BufferSize   11            //Input Buffer to incoming communications
String   InputString = "";          // a string to hold incoming data
boolean  StringComplete = false;    // Flag, whether the string is complete
unsigned long speed = 0;
int inputVoltage = 0;
int inputDirection = 1;

// Définition des paramètres PID
float Kp = 1.828, Ki = 16.4015, Kd = 0; // Ajuste ces valeurs pour un bon comportement
float previousError = 0;
float integral = 0;
float previousTime = 0;

const int motorPin = 9;  // Pin pour contrôler la vitesse du moteur
const int dirPin1 = 12;  // Pin pour la direction 1A
const int dirPin2 = 13;  // Pin pour la direction 2A
const int encoderPin = 2;  // Pin de l'encodeur optique
volatile int encoderCount = 0;  // Compteur d'encoches
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;  // Pour calculer la vitesse
unsigned long Start_time = 0;  // Pour plot la vitesse
unsigned long interval = 50;  // Intervalle pour mesurer la vitesse
float Motor_speed = 0;  // Vitesse du moteur (en tours/minute)
float stepsPerRevolution = 655;

int vmax = 232; // tr/sec

float I_n = 0;
float D_n = 0;
float P_n = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(motorPin, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(encoderPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countEncoder, RISING);  // Détection du front montant
}

void loop() {
  if (StringComplete){
    parseInput();
    speed = map(inputVoltage,0,24,0,255);
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

  // Calculer la vitesse du moteur chaque seconde
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Motor_speed = (encoderCount / stepsPerRevolution) * (60000.0/interval);  // Calcul de la vitesse en tr/min

    float controlSignal = updateControl(Motor_speed);

    float signal = map(controlSignal, 0, vmax, 0, 255);
 
    // Appliquer la nouvelle tension
    analogWrite(motorPin, constrain(signal, 0, 255));

    // Affichage des résultats
    Serial.print((currentMillis - Start_time) / 1000.0);
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

float updateControl(const float Motor_speed) {
  // Calcul PID
  float error = speed - Motor_speed;
  float currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;  // Temps écoulé en secondes
  
  if (deltaTime == 0) deltaTime = 0.01;  // Eviter la division par zéro

  // Proportionnel (P)
  P_n = Kp * error;

  // Intégral (I)
  integral += error * deltaTime;  // Intégration continue
  I_n = Ki * integral;

  // Dérivée (D)
  D_n = Kd * (error - previousError) / deltaTime;  // Taux de variation de l'erreur
  
  // Calcul du signal de commande (tension à appliquer)
  float controlSignal = P_n + I_n + D_n;

  // Sauvegarder l'erreur et le temps pour le prochain cycle
  previousError = error;
  previousTime = currentTime;

  return controlSignal;
}