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

MPU6050Handler mpu;
PeriodicTask mpuTask(5, readMPU6050);


const float resolution = 600;
float r = 0.04;

float K[4] = {-3, -6, -76, -4};

// Commande
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

void setup() {
    
    Serial.println("Setup Started!");
    
    if (!mpu.initialize()) {
        Serial.println("Erreur : Échec de connexion au MPU6050 !");
        while (1);
    }

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
    Serial.println("Setup Completed!");
}

void loop() {

    mpuTask.update();

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

void readMPU6050() {

    Orientation pose = mpu.orientation();

    Serial.print(pose.roll().degree()); Serial.print("\t");
    Serial.print(pose.pitch().degree()); Serial.print("\t");
    Serial.println(pose.yaw().degree());

}

void motorControl() {

    // Calcul de la commande
    u = K[0]*position + K[1]*speed + K[2]*angle + K[3] * angular_vel;

    // constrain(signal, 0, 255) 
    int pwmValue = constrain(map(abs(u), 0, 7, 0, 255), 0, 255);
    pwm[0] = pwmValue;
    pwm[1] = pwmValue;

    digitalWrite(dirPin[0], u <= 0);
    digitalWrite(dirPin[1], u > 0);

    analogWrite(pwmPin[0], pwm[0]);
    analogWrite(pwmPin[1], pwm[1]);
}

