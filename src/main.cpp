#include "I2Cdev.h"
#include "MPU6050.h"
#include "PeriodicTask.h"
#include "MPU6050Handler.h"
#include <MadgwickAHRS.h>
#include "Kalman.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

// Comment this line if we do not want to print on serial port.
#define DEBUG_MODE

#define LED_PIN 13
bool blinkState = false;

void readMPU6050();
void motorControl();
void increaseEncoderCount();

const int interval_motor_task = 10;
const int interval_mpu_task = 5;

Orientation pose;
MPU6050Handler mpu;
PeriodicTask mpuTask(interval_mpu_task, readMPU6050);
PeriodicTask motorTask(interval_motor_task, motorControl);


// Motor properties
const float resolution = 240;
const float diameter = 0.065;
const float radius_m = diameter / 2.0;
const float MAX_VOLTAGE = 12.0;
const float MAX_PWM = 255.0;

// ========== VARIABLES PID ==========
struct PIDAngle {
    float kp = 0.647927870199885; // Coefficient proportionnel
    float ki = 7.29611644063924;   // Coefficient intégral
    float kd = -0.000186631109022805; // Coefficient dérivé
    float integral = 0;
    float lastError = 0;
  } pidAngle;
  
  struct PIDSpeed {
    float kp = -0.0182659464642925; // Coefficient proportionnel
    float ki = -0.0038335764114082;  // Coefficient intégral
    float kd = -0.0193375392442425;  // Coefficient dérivé
    float integral = 0;
    float lastError = 0;
  } pidSpeed;

// Commandes
float u = 0;
float pwm = 0.0;

// Variables d'etat
float position = 0.0;
float speed = 0.0;
float angle = 0.0;
float angular_vel = 0.0;


const int dirPins[2] = {7, 8};
const int pwmPins[2] = {10, 11};  // Pin pour contrôler la vitesse du moteur
const int encoderPins[2] = {2, 3};  // Pin de l'encodeur optique
volatile int encoderCount[2] = {0, 0};  // Compteur d'encoches

void countEncoder_0();
void countEncoder_1();

void setup() {
    
    Serial.println("Setup Started!");
    
    if (!mpu.initialize()) {
        Serial.println("Erreur : Échec de connexion au MPU6050 !");
        while (1);
    }

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);

    pinMode(dirPins[0], OUTPUT);
    pinMode(dirPins[1], OUTPUT);

    pinMode(pwmPins[0], OUTPUT);
    pinMode(pwmPins[1], OUTPUT);

    pinMode(encoderPins[0], INPUT);
    pinMode(encoderPins[1], INPUT);

    attachInterrupt(digitalPinToInterrupt(encoderPins[0]), countEncoder_0, RISING);  // Détection du front montant
    attachInterrupt(digitalPinToInterrupt(encoderPins[1]), countEncoder_1, RISING);  // Détection du front montant

    Serial.println("Setup Completed!");
}

void loop() {

    mpuTask.update();
    motorTask.update();

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

void readMPU6050() {

    pose = mpu.orientation();

    #ifdef DEBUG_MODE
        Serial.print(pose.roll().degree()); Serial.print("\t");
        Serial.print(pose.pitch().degree()); Serial.print("\t");
    #endif
}

void motorControl() {

    // Implementation of PID controls


}

// Fonction d'interruption pour compter les encoches
void countEncoder_0() {
    encoderCount[0]++;
  }
  
  void countEncoder_1() {
    encoderCount[1]++;
  }

  // Provide Steps per seconds
  float steps_per_second(const int encoderCount, const int stepsPerRevolution, const int interval) {
    float speed = (encoderCount / stepsPerRevolution) * (60000.0 / interval);
    return speed;
  }

  // Provide meter per second
  float meters_per_second(const int encoderCount, const int stepsPerRevolution, const int interval, const int diameter) {

    float steps_per_sec = steps_per_second(encoderCount, stepsPerRevolution, interval);

    return steps_per_sec * (PI * diameter) / stepsPerRevolution;
  }


  void applyForce(float force) {
    // Définir un facteur de conversion de force à tension
    float forceToVoltageFactor = 0.198; // Ajustez ce facteur selon votre configuration
  
    // Convertir la force en tension
    float voltage = force * forceToVoltageFactor; // En volts
    int pwm = map(constrain(voltage, 0, MAX_VOLTAGE), 0, MAX_VOLTAGE, 0, MAX_PWM); // Convertir en PWM
  
    // Déterminer la direction
    bool dir = voltage > 0;
    digitalWrite(dirPins[0], dir);
    digitalWrite(dirPins[1], !dir); // Inverser un moteur selon montage
    
    // Appliquer le PWM aux moteurs - A tester
    analogWrite(pwmPins[0], pwm - 30);
    analogWrite(pwmPins[1], pwm);
  }


