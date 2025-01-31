
typedef enum {
    STATE_WAIT,
    STATE_READY,
    STATE_FLIGHT
} FlightState;

static FlightState currentState = STATE_WAIT;


enum CommandType {
    SET, 
    STATE
};

// Struttura Command
struct Command {
    enum CommandType command_type;
    char info[10];
    float value;
};

// Struttura statica per memorizzare l'ultimo comando letto
static struct Command command;



#include <Servo.h>

// Motori
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

static int throttle = 1120;
const int minThrottle = 1120;
const int maxThrottle = 1500;


typedef struct {
    float kp, ki, kd;
    float setpoint;
    float integral, previous_error;
} PIDController;


static PIDController roll_pid;
static PIDController pitch_pid;
static PIDController yaw_pid;
static PIDController altitude_pid;


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
#define TRIGGER_PIN 10
#define ECHO_PIN    11



char* handle_command(){

    switch (command.command_type) {
        case SET:
            set_value_of(command.info, command.value);
            break;

        case STATE:
            flight_fsm_transition(command.info);

        default:
            Serial.println("[DEBUG] Received unknown command type!\n");
            break;
    }

    return;
}


bool read_command() {
    uint8_t byte;
    static int byte_count = 0; // Contatore per il numero di byte letti
    static uint8_t buffer[15]; // Buffer per memorizzare il pacchetto

    while (true) {
        // Leggi un byte dalla UART con un timeout di 100ms
        if (!uart_read_byte(&byte)) {
            return false; // Timeout scaduto, pacchetto non completo
        }

        if (byte_count == 0) {
            // Aspetta il primo byte di inizio pacchetto (0xAA)
            if (byte != 0xAA) {
                continue; // Ignora i byte non validi
            }
        }

        // Salva il byte letto nel buffer
        buffer[byte_count++] = byte;

        // Se il buffer è completo
        if (byte_count == 15) {
            // Ricostruisce la struttura Command dal buffer
            command.command_type = (enum CommandState)buffer[1];

            // Copia i 10 byte successivi nella stringa e aggiunge il terminatore
            memcpy(command.string, &buffer[2], 10);
            command.info[9] = '\0'; // Assicura la terminazione della stringa

            // Ricostruisce il float dal buffer (considerando i 4 byte consecutivi)
            memcpy(&command.value, &buffer[12], 4);

            // Reset del contatore per la prossima lettura
            byte_count = 0;

            return true; // Pacchetto letto correttamente
        }
    }
}




void set_value_of(char* info, float value){
    if(strcmp(info, "ALTITUDE") == 0){
        altitude_pid.setpoint = value;
    } 
    else if(strcmp(info, "THROTTLE") == 0){
        throttle = value;
    } 
    else {
        Serial.println("[DEBUG] Received unknown info type!\n");
    }
}



void flight_fsm_run() {
    switch (currentState) {
    
        case STATE_WAIT:
            Serial.println("[STATE] WAIT");
            handleWait();
            break;
            
        case STATE_READY:
            Serial.println("[STATE] READY");
            handleReady()
            break;
            
        case STATE_FLIGHT:
            Serial.println("[STATE] FLIGHT");
            handleFlight()
            break;
    }
}


void flight_fsm_transition(char* command) {
    switch (currentState) {

        case STATE_WAIT:
            if (strcmp(command, "READY") == 0) {
                currentState = STATE_READY;
            }
            break;
            
        case STATE_READY:
            if (strcmp(command, "START") == 0) {
                currentState = STATE_FLIGHT;
            }
            break;
            
        case STATE_FLIGHT:
            if (strcmp(command, "STOP") == 0) {
                currentState = STATE_WAIT;
            }
            break;
    }
}

void handleWait() {
    Serial.println("[DEBUG] INIT PROCEDURE STARTED");
    
    init_communication();
    init_sensors();
    init_esc();
    init_pid_controllers() 
    
    Serial.println("[DEBUG] INIT PROCEDURE COMPLETED!");    
    
    
    // Change State
    flight_fsm_transition("READY"); 
}

void handleReady(){
    if(read_command()){ handle_command(); }
}

void handleFlight(){
    if(read_command()){ handle_command(); } 
    get_telemetry();
    //drone_control();
    drone_motor();
    send_telemetry();
}



void init_esc(){
  motor1.attach(3);
  motor2.attach(5);
  motor3.attach(6);
  motor4.attach(9);

  Serial.println("[DEBUG] Calibrazione ESC Completata!");
  // Segnale massimo (2000 µs) per iniziare la calibrazione
  /*
  motor1.writeMicroseconds(2000);
  motor2.writeMicroseconds(2000);
  motor3.writeMicroseconds(2000);
  motor4.writeMicroseconds(2000);
  Serial.println("Velocità Max Calibrazione");
  delay(3000); // Attesa per calibrazione
  */
  
  // Segnale minimo (1000 µs) per completare la calibrazione
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  delay(10000); // Attesa per completare

  Serial.println("[DEBUG] Calibrazione ESC Completata!");
}

void motor_power() { 
  motor1.writeMicroseconds(throttle);
  motor2.writeMicroseconds(throttle);
  motor3.writeMicroseconds(throttle);
  motor4.writeMicroseconds(throttle);
  Serial.println("[DEBUG] Motor Throttle: %d", throttle);
}



void init_pid_controllers() {
    pid_init(&roll_pid, 1.0f, 0.0f, 0.0f);
    pid_init(&pitch_pid, 1.0f, 0.0f, 0.0f);
    pid_init(&yaw_pid, 1.0f, 0.0f, 0.0f);
    pid_init(&altitude_pid, 1.0f, 0.0f, 0.0f);
    Serial.println("[DEBUG] PIDs Inizializzati!");

}

void pid_init(PIDController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}

float pid_compute(PIDController *pid, float current_value, float dt) {
    float error = pid->setpoint - current_value;
    pid->integral += error * dt;
    float derivative = (error - pid->previous_error) / dt;
    pid->previous_error = error;

    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}



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

#define START_BYTE 0xAA
#define UART_TIMEOUT 100


// Funzione per leggere un byte dalla UART con timeout
bool uart_read_byte(uint8_t *byte) {
    unsigned long start_time = millis(); // Ottieni il tempo attuale in millisecondi

    // Aspetta fino a quando i dati sono disponibili o il timeout scade
    while (!Serial.available()) {
        if (millis() - start_time >= UART_TIMEOUT) {
            return false; // Timeout scaduto, nessun dato ricevuto
        }
    }

    // Legge un byte dai dati disponibili sulla UART
    *byte = Serial.read();
    return true;
}

// Funzione per inviare un array di byte tramite UART
void uart_write_bytes(uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        Serial.write(data[i]);
    }
}

void init_communication(){
    Serial.begin(9600);
    delay(1000)
}



// Funzione per inviare la telemetria tramite UART
void send_telemetry() {
    uint8_t packet[19]; // 1 byte di start + 4 float (4x4 byte)

    packet[0] = 0xAA; // Byte di start

    // Riempie il pacchetto con i dati di telemetria
    memcpy(&packet[1], &currentTelemetry.roll, 4);
    memcpy(&packet[5], &currentTelemetry.pitch, 4);
    memcpy(&packet[9], &currentTelemetry.yaw, 4);
    memcpy(&packet[13], &currentTelemetry.altitude, 4);

    // Invia il pacchetto tramite UART
    uart_write_bytes(packet, sizeof(packet));
}


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  flight_fsm_run();

}
