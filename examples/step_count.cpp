#include <I2Cdev.h>

//L'encodeur a 100 encoches par tout selon la datasheet

const int encoderPin = 2;  // Pin de l'encodeur
volatile int encoderCount = 0;  // Compteur d'encoches
bool isCounting = true;  // Variable pour indiquer si l'on compte un tour complet

void countEncoder();

void setup() {

  Serial.print("Programme pour compter le nmobre d'encoche dans un tour");
  Serial.begin(115200);
  pinMode(encoderPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countEncoder, RISING);  // Détection du front montant

}

void loop() {
  // Si on est en train de compter un tour, on mesure le nombre d'encoches
  if (isCounting) {
    // Afficher le nombre d'encoches toutes les secondes
    Serial.print("Encoches comptées: ");
    Serial.println(encoderCount);
  }

}

// Fonction d'interruption pour compter les encoches
void countEncoder() {
  encoderCount++;
}
