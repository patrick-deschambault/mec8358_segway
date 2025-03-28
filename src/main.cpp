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

MPU6050Handler mpu;
PeriodicTask sensorTask(100, readMPU6050);

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

    sensorTask.update();

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

void readMPU6050() {

    Orientation pose = mpu.orientation();

    #ifdef OUTPUT_READABLE_ACCELGYRO

        // display tab-separated accel/gyro x/y/z values
        Serial.print(pose.roll().degree()); Serial.print("\t");
        Serial.print(pose.pitch().degree()); Serial.print("\t");
        Serial.println(pose.yaw().degree());
    #endif

    // #ifdef OUTPUT_BINARY_ACCELGYRO
    //     Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
    //     Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
    //     Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
    //     Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
    //     Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
    //     Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    // #endif
}