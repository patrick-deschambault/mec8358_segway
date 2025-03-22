#ifndef MPU6050_HANDLER_H
#define MPU6050_HANDLER_H

#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>

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

#endif // MPU6050_HANDLER_H
