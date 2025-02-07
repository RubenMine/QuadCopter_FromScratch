#ifndef SENSORS_H
#define SENSORS_H

#include <Wire.h>
#include <Adafruit_MPU6050.h>

struct SensorTelemetry {
    float imu_roll, imu_pitch, imu_yaw;
    float accX, accY, accZ;
    float sonar_distance;
};

struct TelemetryData {
    float roll, pitch, yaw;
    float altitude;
};

static struct SensorTelemetry currentSensorTelemetry;
static struct TelemetryData currentTelemetryData;

// Sensor HC-SR04
#define TRIGGER_PIN 10
#define ECHO_PIN    11

// Prototipi delle funzioni
void calibrate_gyro();
void get_gyro_data();
void get_sonar_data();
void get_telemetry();
void init_sensors();
void initGyro();
void initSonar();

extern Adafruit_MPU6050 mpu;

#define PITCH_OFFSET 0  // -3.7
#define ROLL_OFFSET  0  // 0.8

#endif
