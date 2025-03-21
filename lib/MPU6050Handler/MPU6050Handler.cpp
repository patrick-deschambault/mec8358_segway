#include "MPU6050Handler.h"

MPU6050Handler::MPU6050Handler() {}

bool MPU6050Handler::initialize() {

    Serial.begin(115200);

    // initialize device
    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif
    
    // verify connection
    Serial.println("Testing device connections...");
    if (!mpu.testConnection()) {
        return false; // Échec de connexion
    }

    return true; // Connexion réussie
}

Orientation MPU6050Handler::orientation() {

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    return Orientation(filter.getRoll(), filter.getPitch(), filter.getYaw());
}
