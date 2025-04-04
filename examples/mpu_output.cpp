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

const int interval_mpu_task = 5;

MPU6050Handler mpu;
PeriodicTask mpuTask(interval_mpu_task, readMPU6050);

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
    Serial.println(pose.pitch().degree()); Serial.print("\t");
}


