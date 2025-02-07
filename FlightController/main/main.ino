// main.ino
#include <Arduino.h>
#include "pid.h"
#include "flight_fsm.h"
#include "packet.h"
#include "command.h"
//#include <motor.h>
//#include <flight_fsm.h>

typedef struct Settings {
    float throttle;
    int minPWM;
    int maxPWM; 

    bool isAutomatic;
} Settings;

/*
typedef struct States {
    FlightState currentState;

    typedef struct {
    float old_setpoint;
    float new_setpoint;
    } interpol_values interpol;
} States;*/



static Settings setting = {
  .throttle = 0.0,
  .minPWM   = 1120,
  .maxPWM   = 1250,
  .isAutomatic = false
};

void init_communication() {
    Serial.begin(115200);
    delay(1000);
    //sendDebug(F("init_communication: Serial inizializzata a 115200"));
}

/*
void init_esc() {
    motor1.attach(3);
    motor2.attach(5);
    motor3.attach(6);
    motor4.attach(9);
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);
    delay(4000);
    //sendDebug(F("init_esc: ESC inizializzati e timeout completato"));
}
*/

void init_sensors(){
  initSonar();
  initGyro();
}



// Dichiarazione dei PID globali
static PID roll_pid, pitch_pid, yaw_pid, altitude_pid;

/*
void init_pid_controllers() {
    pid_init(&roll_pid, 5, 3.5, 2.5, 200.0f, -200.0f, 0.0f, false);  //kp era 7 ki era 10
    pid_init(&pitch_pid, 5, 3.5, 2.5, 200.0f, -200.0f, 0.0f, false); //kp era 7
    pid_init(&yaw_pid, 5.5, 3.5, 2.5, 200.0f, -200.0f, 0.0f, false);   // kp era 7
    pid_init(&altitude_pid, 14, 20, 16, 12.0f, 8.5f, 0.0f, true);
}*/

void init_pid_controllers() {
    pid_init(&roll_pid, 2, 4, 3, 200.0f, -200.0f, 0.0f, false);  //kp era 7 ki era 10
    pid_init(&pitch_pid, 2, 4, 3, 200.0f, -200.0f, 0.0f, false); //kp era 7
    pid_init(&yaw_pid, 5.5, 3.5, 2.5, 200.0f, -200.0f, 0.0f, false);   // kp era 7
    pid_init(&altitude_pid, 14, 20, 16, 12.0f, 8.5f, 0.0f, true);
}

/*void vTaskComm(void *pvParameters) {
    (void) pvParameters; // Parametro non usato
    for(;;) {
      processIncomingSerial();
    }
}*/

void setup() {
    init_communication();

    /*xTaskCreate(
      vTaskComm,       // Funzione che implementa il task
      "ReadCommand",    // Nome simbolico del task (facoltativo)
      128,            // Dimensione dello stack (in word)
      NULL,           // Parametro da passare al task (se serve)
      1,              // Priorità del task (più alto = più prioritario)
      NULL            // Puntatore all’handle del task (se serve recuperarlo)
    );*/

    sendStatus(STATE_WAIT);
    //sendDebug(F("setup() - Setup completato, sistema in stato WAIT"));
}

void loop() {
    // Gestione dei comandi in ingresso
    //processIncomingSerial();
    
    // Esecuzione della FSM in base allo stato corrente
    flight_fsm_transition("");

    switch (currentState) {
        case STATE_WAIT:
            handleWait();
            break;
        case STATE_READY:
            handleReady();
            break;
        case STATE_HOVER:
            handleHover();
            break;
        case STATE_FLIGHT:
            handleFlight();
            break;
    }
}

