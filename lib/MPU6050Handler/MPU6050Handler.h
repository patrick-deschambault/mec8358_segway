#ifndef MPU6050_HANDLER_H
#define MPU6050_HANDLER_H

#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>



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
    Angle(float degree = 0.0f) {
        angle_deg = degree;
    };
    float degree() {
        return angle_deg;
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
    Madgwick filter;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

public:
    MPU6050Handler(); 
    bool initialize();
    void zero(); 
    Orientation orientation();
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
