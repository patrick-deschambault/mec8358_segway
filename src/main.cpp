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
const int voltage_max = 24;

// ========== VARIABLES PID ==========
struct PIDAngle {
    float kp = 15.0;
    float ki = 2.0;
    float kd = 0.8;
    float integral = 0;
    float lastError = 0;
} pidAngle;

struct PIDSpeed {
    float kp = 0.3;
    float ki = 0.05;
    float kd = 0.1;
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
