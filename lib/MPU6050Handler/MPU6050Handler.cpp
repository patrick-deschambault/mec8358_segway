#include "MPU6050Handler.h"

void i2cWrite(uint8_t address, uint8_t* data, uint8_t length) {
  Wire.beginTransmission(address);
  Wire.write(data, length);
  Wire.endTransmission();
}

void i2cWrite(uint8_t address, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(data);
  Wire.endTransmission();
}

void i2cRead(uint8_t address, uint8_t reg, uint8_t* data, uint8_t length) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false); // Envoi sans stopper la communication

  Wire.requestFrom(address, length);
  for (uint8_t i = 0; i < length; i++) {
      data[i] = Wire.read();
  }
}


MPU6050Handler::MPU6050Handler() {}

bool MPU6050Handler::initialize() {

  Serial.begin(115200);
  Wire.begin();

#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  // Configuration du capteur
  i2cData[0] = 7;    // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  i2cWrite(0x19, i2cData, 4);

  i2cWrite(0x6B, 0x01); // PLL with X axis gyroscope reference and disable sleep mode

  // Vérification du capteur
  i2cRead(0x68, 0x75, i2cData, 1); // Lire le registre WHO_AM_I (0x75)
  if (i2cData[0] != 0x68) {
      Serial.print(F("Error reading sensor"));
      while (1);
  }

  delay(100); // Attente pour stabilisation du capteur

  // Lecture des valeurs initiales de l'accéléromètre
  i2cRead(0x68, 0x3B, i2cData, 6);
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Calcul de l'angle initial
#ifdef RESTRICT_PITCH
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Initialisation du Kalman filter
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

  return true;
}

Orientation MPU6050Handler::orientation() {

    /* Update all the values */
  i2cRead(0x68, 0x3B, i2cData, 14);

  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);

  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  #ifdef RESTRICT_PITCH
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  double gyroXrate = gyroX / 131.0;
  double gyroYrate = gyroY / 131.0;

  #ifdef RESTRICT_PITCH
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
  } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);

  if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate;
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
  } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

  if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate;
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
  #endif

  gyroXangle += gyroXrate * dt;
  gyroYangle += gyroYrate * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;

  return Orientation(Angle(kalAngleX), Angle(kalAngleY), Angle(0));
}

void MPU6050Handler::zero() {

    ZeroData config;

    Initialize(mpu);

    for (int i = config.iAx; i <= config.iGz; i++)
      { // set targets and initial guesses
        config.Target[i] = 0; // must fix for ZAccel 
        config.HighOffset[i] = 0;
        config.LowOffset[i] = 0;
      } // set targets and initial guesses
      config.Target[config.iAz] = 16384;

    SetAveraging(config.NFast, config.N);
    
    PullBracketsOut(config, mpu);

    PullBracketsIn(config, mpu);
    
    Serial.println("-------------- done --------------");
}

void Initialize(MPU6050 &accelgyro)
  {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    Serial.println("PID tuning Each Dot = 100 readings");
  /*A tidbit on how PID (PI actually) tuning works. 
    When we change the offset in the MPU6050 we can get instant results. This allows us to use Proportional and 
    integral of the PID to discover the ideal offsets. Integral is the key to discovering these offsets, Integral 
    uses the error from set-point (set-point is zero), it takes a fraction of this error (error * ki) and adds it 
    to the integral value. Each reading narrows the error down to the desired offset. The greater the error from 
    set-point, the more we adjust the integral value. The proportional does its part by hiding the noise from the 
    integral math. The Derivative is not used because of the noise and because the sensor is stationary. With the 
    noise removed the integral value lands on a solid offset after just 600 readings. At the end of each set of 100 
    readings, the integral value is used for the actual offsets and the last proportional reading is ignored due to 
    the fact it reacts to any noise.
  */
        accelgyro.CalibrateAccel(6);
        accelgyro.CalibrateGyro(6);
        Serial.println("\nat 600 Readings");
        accelgyro.PrintActiveOffsets();
        Serial.println();
        accelgyro.CalibrateAccel(1);
        accelgyro.CalibrateGyro(1);
        Serial.println("700 Total Readings");
        accelgyro.PrintActiveOffsets();
        Serial.println();
        accelgyro.CalibrateAccel(1);
        accelgyro.CalibrateGyro(1);
        Serial.println("800 Total Readings");
        accelgyro.PrintActiveOffsets();
        Serial.println();
        accelgyro.CalibrateAccel(1);
        accelgyro.CalibrateGyro(1);
        Serial.println("900 Total Readings");
        accelgyro.PrintActiveOffsets();
        Serial.println();    
        accelgyro.CalibrateAccel(1);
        accelgyro.CalibrateGyro(1);
        Serial.println("1000 Total Readings");
        accelgyro.PrintActiveOffsets();
     Serial.println("\n\n Any of the above offsets will work nice \n\n Lets proof the PID tuning using another method:"); 
  } // Initialize

  void ForceHeader(int LinesOut)
  { LinesOut = 99; }
    
void GetSmoothed(ZeroData &config, MPU6050 &accelgyro)
  { int16_t RawValue[6];
    int i;
    long Sums[6];
    for (i = config.iAx; i <= config.iGz; i++)
      { Sums[i] = 0; }
//    unsigned long Start = micros();

    for (i = 1; i <= config.N; i++)
      { // get sums
        accelgyro.getMotion6(&RawValue[config.iAx], &RawValue[config.iAy], &RawValue[config.iAz], 
                             &RawValue[config.iGx], &RawValue[config.iGy], &RawValue[config.iGz]);
        if ((i % 500) == 0)
          Serial.print(config.PERIOD);
        delayMicroseconds(config.usDelay);
        for (int j = config.iAx; j <= config.iGz; j++)
          Sums[j] = Sums[j] + RawValue[j];
      } // get sums
//    unsigned long usForN = micros() - Start;
//    Serial.print(" reading at ");
//    Serial.print(1000000/((usForN+N/2)/N));
//    Serial.println(" Hz");
    for (i = config.iAx; i <= config.iGz; i++)
      { config.Smoothed[i] = (Sums[i] + config.N/2) / config.N ; }
  } // GetSmoothed

  void SetOffsets(int TheOffsets[6], ZeroData &config, MPU6050 &accelgyro)
  { accelgyro.setXAccelOffset(TheOffsets [config.iAx]);
    accelgyro.setYAccelOffset(TheOffsets [config.iAy]);
    accelgyro.setZAccelOffset(TheOffsets [config.iAz]);
    accelgyro.setXGyroOffset (TheOffsets [config.iGx]);
    accelgyro.setYGyroOffset (TheOffsets [config.iGy]);
    accelgyro.setZGyroOffset (TheOffsets [config.iGz]);
  } // SetOffsets

void ShowProgress(ZeroData &config)
  { if (config.LinesOut >= config.LinesBetweenHeaders)
      { // show header
        Serial.println("\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
        config.LinesOut = 0;
      } // show header
    Serial.print(config.BLANK);
    for (int i = config.iAx; i <= config.iGz; i++)
      { Serial.print(config.LBRACKET);
        Serial.print(config.LowOffset[i]),
        Serial.print(config.COMMA);
        Serial.print(config.HighOffset[i]);
        Serial.print("] --> [");
        Serial.print(config.LowValue[i]);
        Serial.print(config.COMMA);
        Serial.print(config.HighValue[i]);
        if (i == config.iGz)
          { Serial.println(config.RBRACKET); }
        else
          { Serial.print("]\t"); }
      }
    config.LinesOut++;
  } // ShowProgress

void PullBracketsIn(ZeroData &config, MPU6050 &mpu)
  { boolean AllBracketsNarrow;
    boolean StillWorking;
    int NewOffset[6];
  
    Serial.println("\nclosing in:");
    AllBracketsNarrow = false;
    ForceHeader(config.LinesOut);
    StillWorking = true;
    while (StillWorking) 
      { StillWorking = false;
        if (AllBracketsNarrow && (config.N == config.NFast))
          { SetAveraging(config.NSlow, config.N); }
        else
          { AllBracketsNarrow = true; }// tentative
        for (int i = config.iAx; i <= config.iGz; i++)
          { if (config.HighOffset[i] <= (config.LowOffset[i]+1))
              { NewOffset[i] = config.LowOffset[i]; }
            else
              { // binary search
                StillWorking = true;
                NewOffset[i] = (config.LowOffset[i] + config.HighOffset[i]) / 2;
                if (config.HighOffset[i] > (config.LowOffset[i] + 10))
                  { AllBracketsNarrow = false; }
              } // binary search
          }
        SetOffsets(NewOffset, config, mpu);
        GetSmoothed(config, mpu);
        for (int i = config.iAx; i <= config.iGz; i++)
          { // closing in
            if (config.Smoothed[i] > config.Target[i])
              { // use lower half
                config.HighOffset[i] = NewOffset[i];
                config.HighValue[i] = config.Smoothed[i];
              } // use lower half
            else
              { // use upper half
                config.LowOffset[i] = NewOffset[i];
                config.LowValue[i] = config.Smoothed[i];
              } // use upper half
          } // closing in
        ShowProgress(config);
      } // still working
   
  } // PullBracketsIn

void PullBracketsOut(ZeroData &config, MPU6050 &mpu)
  { boolean Done = false;
    int NextLowOffset[6];
    int NextHighOffset[6];

    Serial.println("expanding:");
    ForceHeader(config.LinesOut);
 
    while (!Done)
      { Done = true;
        SetOffsets(config.LowOffset, config, mpu);
        GetSmoothed(config, mpu);
        for (int i = config.iAx; i <= config.iGz; i++)
          { // got low values
            config.LowValue[i] = config.Smoothed[i];
            if (config.LowValue[i] >= config.Target[i])
              { Done = false;
                NextLowOffset[i] = config.LowOffset[i] - 1000;
              }
            else
              { NextLowOffset[i] = config.LowOffset[i]; }
          } // got low values
      
        SetOffsets(config.HighOffset, config, mpu);
        GetSmoothed(config, mpu);
        for (int i = config.iAx; i <= config.iGz; i++)
          { // got high values
            config.HighValue[i] = config.Smoothed[i];
            if (config.HighValue[i] <= config.Target[i])
              { Done = false;
                NextHighOffset[i] = config.HighOffset[i] + 1000;
              }
            else
              { NextHighOffset[i] = config.HighOffset[i]; }
          } // got high values
        ShowProgress(config);
        for (int i = config.iAx; i <= config.iGz; i++)
          { config.LowOffset[i] = NextLowOffset[i];   // had to wait until ShowProgress done
            config.HighOffset[i] = NextHighOffset[i]; // ..
          }
     } // keep going
  } // PullBracketsOut

void SetAveraging(int NewN, int &N)
  { N = NewN;
    Serial.print("averaging ");
    Serial.print(N);
    Serial.println(" readings each time");
   } // SetAveraging