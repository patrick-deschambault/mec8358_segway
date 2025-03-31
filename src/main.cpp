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

#define LED_PIN 13
bool blinkState = false;

void readMPU6050();
void motorControl();
void increaseEncoderCount();

const int interval_motor_task = 10;
const int interval_mpu_task = 5;


MPU6050Handler mpu;
PeriodicTask mpuTask(interval_mpu_task, readMPU6050);
PeriodicTask motorTask(interval_motor_task, motorControl);


// Motor properties
const float resolution = 600;
const float radius_m = 0.04;
const int voltage_max = 24;

// Gains
float K[4] = {-3, -6, -76, -4};

// Commandes
float u = 0;
float pwm[2] = {0.0, 0.0};

// Variables d'etat
float position = 0.0;
float speed = 0.0;
float angle = 0.0;
float angular_vel = 0.0;

// Pins d'entree
const int pwmPin[2] = {10, 11};
const int dirPin[2] = {8, 9};
const int encoderPin = 2;

// Status de l'encodeur optique
int encoderCount = 0;

void setup() {
    
    Serial.println("Setup Started!");
    
    if (!mpu.initialize()) {
        Serial.println("Erreur : Échec de connexion au MPU6050 !");
        while (1);
    }

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);

    pinMode(dirPin[0], OUTPUT);
    pinMode(dirPin[1], OUTPUT);
    pinMode(pwmPin[0], OUTPUT);
    pinMode(pwmPin[1], OUTPUT);

    pinMode(encoderPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPin), increaseEncoderCount, RISING);  // Détection du front montant

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

    Orientation pose = mpu.orientation();

    Serial.print(pose.roll().degree()); Serial.print("\t");
    Serial.print(pose.pitch().degree()); Serial.print("\t");
    Serial.print("\n");

}

void motorControl() {

    position = position + ((encoderCount*2*PI*radius_m) / (resolution));

    speed = ((float)encoderCount * 1000.0 * 2.0 * PI * radius_m) / (resolution * interval_motor_task);

    angle = mpu.orientation().pitch().to_radians();

    angular_vel = mpu.gyroYAngle().radians_per_sec();

    // Calcul de la commande
    u = K[0]*position + K[1]*speed + K[2]*angle + K[3]*angular_vel;

    // constrain(signal, 0, 255) 
    int pwmValue = constrain(map(abs(u), 0, voltage_max, 0, 255), 0, 255);
    pwm[0] = pwmValue;
    pwm[1] = pwmValue;

    digitalWrite(dirPin[0], u <= 0);
    digitalWrite(dirPin[1], u > 0);

    analogWrite(pwmPin[0], pwm[0]);
    analogWrite(pwmPin[1], pwm[1]);

    encoderCount = 0;
}

void increaseEncoderCount() {
    encoderCount++;
}
