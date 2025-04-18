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
#define DEBUG_MODE 1

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
    float kp = 1.15; // Coefficient proportionnel
    float ki = 7.0;   // Coefficient intégral
    float kd = 0.05; // Coefficient dérivé
    float integral = 0;
    float lastError = 0;
  } pidAngle;
  
  struct PIDSpeed {
    float kp = 0.15; // Coefficient proportionnel
    float ki = 0.001;  // Coefficient intégral
    float kd = 0.001;  // Coefficient dérivé
    float integral = 0;
    float lastError = 0;
  } pidSpeed;

// Commandes
int pwm = 0;

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
    Serial.begin(115200);
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
unsigned long lastTime = 0;
const unsigned long speedCalcInterval = 10;
void main_loop() {
    
    Orientation pose = mpu.orientation();

    // Vérifiez si l'intervalle de calcul de la vitesse est écoulé
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= speedCalcInterval) {
        Serial.print("Counts: "); Serial.print(encoderCount[0]); Serial.print(" | "); Serial.println(encoderCount[1]);

        motor_speeds[0] = meters_per_second(encoderCount[0], resolution, speedCalcInterval, diameter);
        motor_speeds[1] = meters_per_second(encoderCount[1], resolution, speedCalcInterval, diameter);

        float average_speed = (motor_speeds[0] + motor_speeds[1]) / 2.0;

        encoderCount[0] = 0;
        encoderCount[1] = 0;

        // Lire l'angle
        float rollAngle = pose.roll().degree();


        // PID angle -> target force
        float target_force = computeAnglePID(rollAngle, speedCalcInterval / 1000.0, average_speed);

        // PID vitesse -> force corrigée
        float forceCmd = computeSpeedPID(target_force, average_speed, speedCalcInterval / 1000.0);

        applyForce(forceCmd);

        lastTime = currentTime;

        #ifdef DEBUG_MODE
            Serial.print(rollAngle); Serial.print("\t");
            Serial.print(pose.pitch().degree()); Serial.print("\t");
            Serial.print(motor_speeds[0]); Serial.print("\t");
            Serial.print(motor_speeds[1]); Serial.print("\t");
            Serial.print(average_speed); Serial.print("\t");
            Serial.print(target_force); Serial.print("\t");
            Serial.print(forceCmd); Serial.print("\t");
            Serial.println(pwm);
        #endif
    }
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
    float speed = ((float)encoderCount / (float)stepsPerRevolution) * (60000.0 / interval);
    return speed;
}

  // Provide meter per second
  float meters_per_second(const int encoderCount, const int stepsPerRevolution, const int interval, const float diameter){

    float steps_per_sec = steps_per_second(encoderCount, stepsPerRevolution, interval);
    return steps_per_sec * (PI * diameter) / stepsPerRevolution;
}

  float computeAnglePID(float angle, float dt, float vitesseLineaire) {
    // Calcul de l'erreur en tenant compte de la vitesse linéaire
    float gain_vitesse = 15; // Ajustez ce gain selon vos besoins
    float error = 0 - (angle - (vitesseLineaire * gain_vitesse));  // Cible = 0°
  
    pidAngle.integral += error * dt;
    pidAngle.integral = constrain(pidAngle.integral, -10, 10);
  
    float deriv = (error - pidAngle.lastError) / dt;
    pidAngle.lastError = error;
  
    return pidAngle.kp * error + 
                    pidAngle.ki * pidAngle.integral + 
                    pidAngle.kd * deriv;
  }

  float computeSpeedPID(float targetForce, float vitesseLineaire, float dt) {
    
    float gain_vitesse = 35; // Ajustez ce gain selon vos besoins

    // Calcul de l'erreur pour le PID de vitesse
    float error = targetForce - (vitesseLineaire * gain_vitesse); // Cible = vitesse linéaire souhaitée
  
    pidSpeed.integral += error * dt;
    pidSpeed.integral = constrain(pidSpeed.integral, -10, 10);
  
    float deriv = (error - pidSpeed.lastError) / dt;
    pidSpeed.lastError = error;
  
    return pidSpeed.kp * error + 
                     pidSpeed.ki * pidSpeed.integral + 
                     pidSpeed.kd * deriv;
  }
  
  void applyForce(float force) {

    // Définir un facteur de conversion de force à tension
    float forceToVoltageFactor = 40 / MAX_VOLTAGE; // 30N de force à pleine charge donc 30N / volt max

    // Convertir la force en tension
    float voltage = force * forceToVoltageFactor; // En volts
    pwm = map(voltage, 0, MAX_VOLTAGE, 0, MAX_PWM); // Convertir en PWM
    
    // Déterminer la direction
    bool dir = voltage > 0;
    digitalWrite(dirPins[0], dir);
    digitalWrite(dirPins[1], !dir);
    
    // Calculer le PWM, en prenant la valeur absolue de la tension
    voltage = abs(voltage);
    voltage = constrain(voltage, 0, MAX_VOLTAGE);
    pwm = map(voltage, 0, MAX_VOLTAGE, 0, MAX_PWM);
    
    // Appliquer le PWM
    analogWrite(pwmPins[0], pwm * 0.98);
    analogWrite(pwmPins[1], pwm);
}

