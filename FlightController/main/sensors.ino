#include "sensors.h"

// Sensor GY-87
Adafruit_MPU6050 mpu;
float pitch = 0.0, roll = 0.0, yaw = 0.0;
float alpha = 0.90; // Costante del filtro complementare
float gyroXOffset = 0.0, gyroYOffset = 0.0, gyroZOffset = 0.0;
unsigned long lastTime = 0;

void initGyro(){
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Errore: MPU6050 non trovato!");
    while (1);
  }
  Serial.println("[DEBUG] MPU6050 inizializzato!");

  calibrate_gyro();

  // Inizializza pitch e roll con l'accelerometro
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  roll = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  yaw = yaw; //needed magnetometer
}


void initSonar() {
  // -------------------------
  // Inizializzazione HC-SR04
  // -------------------------
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  // Metti il trigger a LOW di default
  digitalWrite(TRIGGER_PIN, LOW);

  Serial.println("[DEBUG] HC-SR04 inizializzato!");
}


void calibrate_gyro() {
  Serial.println("Calibrazione del giroscopio...");
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 1000;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(1);
  }

  gyroXOffset = sumX / samples;
  gyroYOffset = sumY / samples;
  gyroZOffset = sumZ / samples;

  //char msg[100];
  //snprintf(msg, sizeof(msg), "calibrate_gyro: Offsets: X=%f, Y=%f, Z=%f", gyroXOffset, gyroYOffset, gyroZOffset);
  //sendDebug(msg);
}



void get_gyro_data() {
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0; // In secondi
  lastTime = currentTime;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accelPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  float accelRoll = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  pitch += (g.gyro.y - gyroYOffset) * dt * 180 / PI;
  roll += (g.gyro.x - gyroXOffset) * dt * 180 / PI;
  yaw += (g.gyro.z - gyroZOffset) * dt * 180 / PI;

  pitch = alpha * pitch + (1.0 - alpha) * accelPitch;
  roll = alpha * roll + (1.0 - alpha) * accelRoll;

  pitch = normalizeAngle(pitch);
  roll = normalizeAngle(roll);
  yaw = normalizeAngle(yaw);


  currentTelemetryData.roll = roll - ROLL_OFFSET; 
  currentTelemetryData.pitch = pitch - PITCH_OFFSET;
  currentTelemetryData.yaw = yaw;

  delay(1);
}


void get_sonar_data() {
    // Definizione dei pin del sonar
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    unsigned long duration = pulseIn(ECHO_PIN, HIGH);
    float distanceCm = duration * 0.0343f / 2.0f;
    float pitchRad = currentTelemetryData.pitch * PI / 180.0f;
    float rollRad  = currentTelemetryData.roll  * PI / 180.0f;
    float altCm    = distanceCm * cos(pitchRad) * cos(rollRad);
    currentTelemetryData.altitude = (altCm - 5) / 100; // in metri
    
    //char msg[80];
    //snprintf(msg, sizeof(msg), "get_sonar_data: duration=%lu, distanceCm=%f, altitude=%f", duration, distanceCm, currentTelemetryData.altitude);
    //sendDebug(msg);
}


float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

void get_telemetry() {
  get_sonar_data();
  get_gyro_data();
}
