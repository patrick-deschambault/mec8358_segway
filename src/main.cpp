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

void main_loop();
void increaseEncoderCount();
void applyForce(float);
float computeSpeedPID(float, float , float);
float computeAnglePID(float, float, float );
float steps_per_second(const int , const int, const int );
float meters_per_second(const int , const int , const int , const int );


const int interval = 10;

MPU6050Handler mpu;
PeriodicTask main_task(interval, main_loop);


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


float motor_speeds[2] = {0.0, 0.0};
int pwmPins[2] = {10, 11};  // Pin pour contrôler la vitesse du moteur

const int dirPins[2] = {7, 8};
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

    main_task.update();

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

void main_loop() {

    Orientation pose = mpu.orientation();

    // Implementation of PID controls
    motor_speeds[0] = meters_per_second(encoderCount[0], resolution, interval, diameter);
    motor_speeds[1] = meters_per_second(encoderCount[1], resolution, interval, diameter);

    float average_speed = (motor_speeds[0] + motor_speeds[1]) / 2.0;

    float target_force = computeAnglePID(pose.roll().degree(), interval, average_speed);

    float forceCmd = computeSpeedPID(target_force, average_speed, interval);

    applyForce(forceCmd);

    #ifdef DEBUG_MODE
        Serial.print(pose.roll().degree()); Serial.print("\t");
        Serial.println(pose.pitch().degree());
    #endif

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

  float computeAnglePID(float angle, float dt, float vitesseLineaire) {
    // Calcul de l'erreur en tenant compte de la vitesse linéaire
    float gain_vitesse = 10; // Ajustez ce gain selon vos besoins
    float error = 0 - (angle - (vitesseLineaire * gain_vitesse));  // Cible = 0°
  
    pidAngle.integral += error * dt;
    pidAngle.integral = constrain(pidAngle.integral, -5, 5);
  
    float deriv = (error - pidAngle.lastError) / dt;
    pidAngle.lastError = error;
  
    return constrain(pidAngle.kp * error + 
                    pidAngle.ki * pidAngle.integral + 
                    pidAngle.kd * deriv, -10, 10);
  }

  float computeSpeedPID(float targetForce, float vitesseLineaire, float dt) {
    
    float gain_vitesse = 10; // Ajustez ce gain selon vos besoins

    // Calcul de l'erreur pour le PID de vitesse
    float error = targetForce - (vitesseLineaire * gain_vitesse); // Cible = vitesse linéaire souhaitée
  
    pidSpeed.integral += error * dt;
    pidSpeed.integral = constrain(pidSpeed.integral, -50, 50);
  
    float deriv = (error - pidSpeed.lastError) / dt;
    pidSpeed.lastError = error;
  
    return constrain(pidSpeed.kp * error + 
                     pidSpeed.ki * pidSpeed.integral + 
                     pidSpeed.kd * deriv, -MAX_VOLTAGE, MAX_VOLTAGE);
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


