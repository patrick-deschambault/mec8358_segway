#ifndef MPU6050_HANDLER_H
#define MPU6050_HANDLER_H

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>
#include "Kalman.h"

struct ZeroData {
    // Constantes
    const char LBRACKET = '[';
    const char RBRACKET = ']';
    const char COMMA    = ',';
    const char BLANK    = ' ';
    const char PERIOD   = '.';

    const int iAx = 0;
    const int iAy = 1;
    const int iAz = 2;
    const int iGx = 3;
    const int iGy = 4;
    const int iGz = 5;

    const int usDelay = 3150;
    const int NFast =  1000;
    const int NSlow = 10000;
    const int LinesBetweenHeaders = 5;

    // Tableaux
    int LowValue[6] = {0};  // Initialisation par défaut
    int HighValue[6] = {0};
    int Smoothed[6] = {0};
    int LowOffset[6] = {0};
    int HighOffset[6] = {0};
    int Target[6] = {0};

    int LinesOut = 0;
    int N = 0;
};

class Angle {

    private: 
    float angle_deg;

    public:
    Angle(double degree = 0.0) {
        angle_deg = degree;
    };
    float degree() {
        return angle_deg;
    };
    float to_radians() {
        return angle_deg * DEG_TO_RAD;
    };
};

class AngularVelocity {

    private:
    float vel_deg_per_sec;

    public:
    AngularVelocity(float value_degree_per_sec = 0.0f) {
        vel_deg_per_sec = value_degree_per_sec;
    };
    float radians_per_sec() {
        return vel_deg_per_sec * DEG_TO_RAD;
    };
};

class Orientation {

    private:
    Angle angles[3];

    public:

    Orientation(Angle roll = Angle(), Angle pitch = Angle(), Angle yaw = Angle()) {
        angles[0] = roll;
        angles[1] = pitch;
        angles[2] = yaw;
    };
    Angle roll() {
        return angles[0];
    };
    Angle pitch() {
        return angles[1];
    };
    Angle yaw() {
        return angles[2];
    };
};

class MPU6050Handler {
private:
    MPU6050 mpu;

    Kalman kalmanX; // Create the Kalman instances
    Kalman kalmanY;

    uint32_t timer;
    uint8_t i2cData[14]; // Buffer for I2C data
    int16_t tempRaw;
    double accX, accY, accZ;
    double gyroX, gyroY, gyroZ;
    double gyroXangle, gyroYangle; // Angle calculate using the gyro only
    double compAngleX, compAngleY; // Calculated angle using a complementary filter
    double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

public:
    MPU6050Handler(); 
    bool initialize();
    void zero(); 
    Orientation orientation();
    AngularVelocity gyroYAngle();
};

void Initialize(MPU6050&);
void SetAveraging(int NewN, int &N);
void PullBracketsOut(ZeroData &config, MPU6050 &mpu);
void PullBracketsIn(ZeroData &config, MPU6050 &mpu);
void ShowProgress(ZeroData &config);
void SetOffsets(int TheOffsets[6], ZeroData &config, MPU6050 &accelgyro);
void GetSmoothed(ZeroData &config, MPU6050 &accelgyro);
void ForceHeader(int LinesOut);

#endif // MPU6050_HANDLER_H
