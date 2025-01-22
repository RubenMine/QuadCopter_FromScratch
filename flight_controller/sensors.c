#include "header/sensors.h"

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
  yaw = yaw //needed magnetometer
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


void init_sensors(){
  initSonar();
  initGyro();
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

  pitch += (g.gyro.y - gyroYOffset) * dt * 180 / PI;
  roll += (g.gyro.x - gyroXOffset) * dt * 180 / PI;
  yaw += (g.gyro.z - gyroZOffset) * dt * 180 / PI;

  pitch = alpha * pitch + (1.0 - alpha) * accelPitch;
  roll = alpha * roll + (1.0 - alpha) * accelRoll;

  pitch = normalizeAngle(pitch);
  roll = normalizeAngle(roll);
  yaw = normalizeAngle(yaw);


  currentTelemetryData.roll = roll; 
  currentTelemetryData.pitch = pitch;
  currentTelemetryData.yaw = yaw;
  //currentTelemetry.altitude = 100.5;  // Valore simulato

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

float readSonarDistanceHCSR04() {
  // Assicurati che TRIGGER sia basso per almeno 2 microsecondi
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);

  // Manda un impulso HIGH di 10 microsecondi per avviare la misura
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  // Misura il tempo (in microsecondi) in cui ECHO rimane HIGH
  unsigned long duration = pulseIn(ECHO_PIN, HIGH);

  // Converte il tempo in distanza
  // Velocità del suono ~340 m/s -> 29.412 microsecondi per cm (andata e ritorno)
  // Dividere per 2 per ottenere solo la distanza "di andata".
  float distanceCm = duration * 0.0343 / 2.0;  

  return distanceCm;  
}


void get_sonar_data() {
  // Legge la distanza dal sensore HC-SR04
  float rawDistanceCm = readSonarDistanceHCSR04();
  if (rawDistanceCm <= 0.0) {
    // Se la lettura non è valida, gestisci l'errore come preferisci
    return;
  }


  // Converte da gradi a radianti
  float pitchRad = currentTelemetryData.pitch * PI / 180.0;
  float rollRad  = currentTelemetryData.roll  * PI / 180.0;

  // Approssimazione: se il drone è inclinato,
  // altitudine effettiva = distanza letta * cos(pitch) * cos(roll)
  // (Assumendo piccole inclinazioni e che l'asse del sensore punti verso il suolo quando 0°)
  float altitudeCm = rawDistanceCm * cos(pitchRad) * cos(rollRad);

  // Salva nei dati di telemetria
  currentTelemetryData.altitude = altitudeCm;  // in cm

  // Per debug
  Serial.print("Distanza (HC-SR04): ");
  Serial.print(rawDistanceCm);
  Serial.print(" cm | Altitudine corretta: ");
  Serial.print(altitudeCm);
  Serial.println(" cm");
}



float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}
