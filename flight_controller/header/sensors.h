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


// Sensor GY-87
Adafruit_MPU6050 mpu;
float pitch = 0.0, roll = 0.0, yaw = 0.0;
float alpha = 0.85; // Costante del filtro complementare
float gyroXOffset = 0.0, gyroYOffset = 0.0, gyroZOffset = 0.0;
unsigned long lastTime = 0;


// Sensor HC-SR04
#define TRIGGER_PIN 2
#define ECHO_PIN    3