#include <Wire.h>
#include <Adafruit_MPU6050.h>

struct TelemetryData {
    float roll;
    float pitch;
    float yaw;
    float altitude;
};
struct TelemetryData currentTelemetry;

Adafruit_MPU6050 mpu;

float pitch = 0.0, roll = 0.0, yaw = 0.0;
float alpha = 0.80; // Costante del filtro complementare
float gyroXOffset = 0.0, gyroYOffset = 0.0, gyroZOffset = 0.0;
unsigned long lastTime = 0;

void init_sensors(){
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Errore: MPU6050 non trovato!");
    while (1);
  }
  Serial.println("MPU6050 inizializzato!");

  calibrate_gyro();

  // Inizializza pitch e roll con l'accelerometro
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  roll = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  /*  
  Serial.print("Pitch iniziale: ");
  Serial.println(pitch);
  Serial.print("Roll iniziale: ");
  Serial.println(roll);
  */
  lastTime = micros();
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

  Serial.println("Calibrazione completata.");
  Serial.print("Offset X: "); Serial.println(gyroXOffset);
  Serial.print("Offset Y: "); Serial.println(gyroYOffset);
  Serial.print("Offset Z: "); Serial.println(gyroZOffset);
  delay(1000);
}

void get_gyro_data() {
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0; // In secondi
  lastTime = currentTime;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accelPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  float accelRoll = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  pitch += (g.gyro.y - gyroYOffset) * dt;
  roll += (g.gyro.x - gyroXOffset) * dt;
  yaw += (g.gyro.z - gyroZOffset) * dt;

  pitch = alpha * pitch + (1 - alpha) * accelPitch;
  roll = alpha * roll + (1 - alpha) * accelRoll;

  pitch = normalizeAngle(pitch);
  roll = normalizeAngle(roll);
  yaw = normalizeAngle(yaw);


  currentTelemetry.roll = roll; 
  currentTelemetry.pitch = pitch;
  currentTelemetry.yaw = yaw;
  currentTelemetry.altitude = 100.5;  // Valore simulato

  /*
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print(" Roll: ");
  Serial.print(roll);
  Serial.print(" Yaw: ");
  Serial.println(yaw);
  */

  delay(1);
}

float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}
