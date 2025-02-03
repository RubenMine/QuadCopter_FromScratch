#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>


// Sensor HC-SR04
#define TRIGGER_PIN 10
#define ECHO_PIN    11
#define START_BYTE 0xAA
#define UART_TIMEOUT 100


//Strutture critiche 

typedef enum {
    STATE_WAIT,
    STATE_READY,
    STATE_FLIGHT, 
    STATE_HOVER
} FlightState;


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

typedef struct {
    int Kp, Ki, Kd;
    float Integrator_STATE, Filter_STATE;
    float setpoint, output;    
    float UpperLimit, LowerLimit;
    bool anti_windup;
} PID;


typedef struct {
    float old_setpoint;
    float new_setpoint;
} interpol_values;

struct SensorTelemetry {
    float imu_roll, imu_pitch, imu_yaw;
    float accX, accY, accZ;
    float sonar_distance;
};

struct TelemetryData {
    float roll, pitch, yaw;
    float altitude;
};

typedef struct {
    int pw_min;
    int pw_max;
    int pwm_min;
    int pwm_max;
} power;

static power pw;
static FlightState currentState = STATE_WAIT;

// Struttura statica per memorizzare l'ultimo comando letto
static struct Command command;

static interpol_values v;

float t;

//PID
static PID roll_pid;
static PID pitch_pid;
static PID yaw_pid;
static PID altitude_pid;

// Motori
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

static struct SensorTelemetry currentSensorTelemetry;
static struct TelemetryData currentTelemetryData;


// Sensor GY-87
Adafruit_MPU6050 mpu;
float pitch = 0.0, roll = 0.0, yaw = 0.0;
float alpha = 0.85; // Costante del filtro complementare
float gyroXOffset = 0.0, gyroYOffset = 0.0, gyroZOffset = 0.0;
unsigned long lastTime = 0;


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
        v.new_setpoint = value;
        v.old_setpoint = altitude_pid.setpoint;
    } 
    else if(strcmp(info, "THROTTLE") == 0){
        throttle = value;
    } 
    else {
        Serial.println("[DEBUG] Received unknown info type!\n");
    }
}


void flight_fsm_run() {
    flight_fsm_transition(NULL);

    switch (currentState) {
        case STATE_WAIT:
            Serial.println("[STATE] WAIT");
            handleWait();
            break;
            
        case STATE_READY:
            Serial.println("[STATE] READY");
            handleReady();
            break;
            
        case STATE_HOVER:
            Serial.println("[STATE] HOVER");
            handleHover();
            break;

        case STATE_FLIGHT:
            Serial.println("[STATE] FLIGHT");
            handleFlight();
            break;
    }
}


void flight_fsm_transition(char* command = NULL) {
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
            if(currentTelemetryData.altitude>altitude_pid.setpoint-0.05 && currentTelemetryData.altitude<altitude_pid.setpoint+0.05) {
                currentState = STATE_HOVER;
            }
            break;

        case STATE_HOVER:
            if(currentTelemetryData.altitude<altitude_pid.setpoint-0.05 || currentTelemetryData.altitude>altitude_pid.setpoint+0.05) {
                currentState = STATE_FLIGHT;
            }
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


// ALEALE IMPLEMENTATION
float normalize_angles(float angle) {
    return -1 + (angle+6)/7;
}

float normalize_thrust(float thrust) {
    return (thrust-8.5f)/3.5f;
}

float norm_to_physical(float m_norm) {
    return pw->pw_min + (m_norm)*(pw->pw_max-pw->pw_max);
}

void set_range_power(int pwm_min, int pwm_max) {
    pw->pw_min = respective_power(pwm_min);
    pw->pw_max = respective_power(pwm_max);
    pw->pwm_min = pwm_min;
    pw->pwm_max = pwm_max;
}

int respective_power(int pwm) {
    return (pwm-1000)*(1175)/1000;
}

float pw_to_pwm(float p) {
    return pw->pwm_min + (p-pw->pw_min)*(pw->pwm_max-pw->pwm_min)/(pw->pw_max-pw->pw_min);
}

float interpolate(int h_start, int h_end, float t_diff) {
    float T_trans = 4 * abs(h_end-h_start);
    float h;
    if(t_diff<=T_trans) h = h_start + (h_end - h_start)/2 * (1 - cos(atan(1)*4* (t_diff)/T_trans));
    else h = h_end;
    return h;
}

drone_control(){
    pid_compute(&altitude_pid, currentTelemetryData.altitude);
    pid_compute(&roll_pid, currentTelemetryData.roll);
    pid_compute(&pitch_pid, currentTelemetryData.pitch);
    pid_compute(&yaw_pid, currentTelemetryData.yaw);

    float throttle, roll, yaw, pitch;
    normalized_throttle = normalize_thrust(altitude_pid.output);
    normalized_roll = normalize_angles(roll_pid.output);
    normalized_yaw = normalize_angles(yaw_pid.output);
    normalized_pitch = normalize_angles(pitch_pid.output);

    float M1, M2, M3, M4;    
    float k_pitch = 0.5, k_roll = 0.5, k_yaw = 1.0;
    M1 = normalized_throttle + k_pitch * normalized_pitch + k_roll * normalized_roll + k_yaw * normalized_yaw;
    M2 = normalized_throttle + k_pitch * normalized_pitch - k_roll * normalized_roll - k_yaw * normalized_yaw;
    M3 = normalized_throttle - k_pitch * normalized_pitch + k_roll * normalized_roll + k_yaw * normalized_yaw;
    M4 = normalized_throttle - k_pitch * normalized_pitch - k_roll * normalized_roll - k_yaw * normalized_yaw;

    float MF1, MF2, MF3, MF4;
    MF1 = norm_to_physical(M1);
    MF2 = norm_to_physical(M2);
    MF3 = norm_to_physical(M3);
    MF4 = norm_to_physical(M4);

    float PWM1, PWM2, PWM3, PWM4;
    PWM1 = pw_to_pwm(MF1);
    PWM2 = pw_to_pwm(MF2);
    PWM3 = pw_to_pwm(MF3);
    PWM4 = pw_to_pwm(MF4);

    motor_power(PWM1, PWM2, PWM3, PWM4);
}

void handleFlight(){
    if(read_command()){ handle_command(); } 
    get_telemetry();

    float alt_curr_setpoint = interpolate(v.old_setpoint, v.new_setpoint, t);
    altitude_pid.setpoint = alt_curr_setpoint;
    drone_control();

    send_telemetry();
}

void handleHover() {
    if(read_command()){ handle_command(); } 
    get_telemetry();
    drone_control();
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

void motor_power(int M1, int M2, int M3, int M4) { 
  motor1.writeMicroseconds(M1);
  motor2.writeMicroseconds(M2);
  motor3.writeMicroseconds(M3);
  motor4.writeMicroseconds(M4);

  Serial.println(M1, M2, M3, M4);
}

void pid_init(PID* pid, int kp, int ki, int kd, float UpperLimit, float LowerLimit, float setpoint, bool a_windup) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Filter_STATE = 0.0f;
    pid->Integrator_STATE = 0.0f;
    pid->UpperLimit = UpperLimit;
    pid->LowerLimit = LowerLimit;
    pid->setpoint = setpoint;
    pid->anti_windup = a_windup;
}


void init_pid_controllers() {
    pid_init(&roll_pid, 7, 10, 5, 0.0f, -10.0f, 0, false);
    pid_init(&pitch_pid, 7, 10, 5, 0.0f, -10.0f, 0, false);
    pid_init(&yaw_pid, 7, 10, 5, 0.0f, -10.0f, 0, false);
    pid_init(&altitude_pid, 14.0f, 20.0f, 16.0f, 10.0f, 8.5f, 0, true);
    Serial.println("[DEBUG] PIDs Inizializzati!");

}



void pid_compute(PID* pid, float sensor_value){
  float FilterCoefficient;
  float Sum;
  float error = pid->setpoint-sensor_value;

  /* Gain: '<S36>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S28>/Filter'
   *  Gain: '<S27>/Derivative Gain'
   *  Inport: '<Root>/u'
   *  Sum: '<S28>/SumD'
   */
  FilterCoefficient = (pid->Kd * error - pid->Filter_STATE) *100.0;
  //printf("%f\n", FilterCoefficient);

  /* Sum: '<S43>/Sum' incorporates:
   *  DiscreteIntegrator: '<S33>/Integrator'
   *  Gain: '<S38>/Proportional Gain'
   *  Inport: '<Root>/u'
   */
  Sum = (pid->Kp * error + pid->Integrator_STATE) +
    FilterCoefficient;
  //printf("%f\n", Sum);

  /* Switch: '<S41>/Switch2' incorporates:
   *  Inport: '<Root>/LowerLimit'
   *  Inport: '<Root>/UpperLimit'
   *  RelationalOperator: '<S41>/LowerRelop1'
   *  RelationalOperator: '<S41>/UpperRelop'
   *  Switch: '<S41>/Switch'
   */
  if (Sum > pid->UpperLimit) {
    /* Switch: '<S41>/Switch2' */
    pid->output = pid->UpperLimit;
  } else if (Sum < pid->LowerLimit) {
    /* Switch: '<S41>/Switch2' incorporates:
     *  Inport: '<Root>/LowerLimit'
     *  Switch: '<S41>/Switch'
     */
    pid->output = pid->LowerLimit;
  } else {
    /* Switch: '<S41>/Switch2' incorporates:
     *  Switch: '<S41>/Switch'
     */
    pid->output = Sum;
  }
  //printf("%f\n", pid->output);

  /* End of Switch: '<S41>/Switch2' */

  /* Update for DiscreteIntegrator: '<S33>/Integrator' incorporates:
   *  Gain: '<S26>/Kb'
   *  Gain: '<S30>/Integral Gain'
   *  Inport: '<Root>/u'
   *  Sum: '<S26>/SumI2'
   *  Sum: '<S26>/SumI4'
   */
  float anti_windup;
  if(pid->anti_windup) anti_windup = (pid->output - Sum) * 2.0;
  else anti_windup = 0;

  pid->Integrator_STATE += (anti_windup + pid->Ki * error) * 0.01;
  
  /* Update for DiscreteIntegrator: '<S28>/Filter' */
  pid->Filter_STATE += 0.01 * FilterCoefficient;
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
