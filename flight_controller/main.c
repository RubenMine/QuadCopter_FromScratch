#include <Arduino.h>
#include <stdio.h>
#include "src/uart_comm.c"
#include "src/sensors.c"

void setup() {
    init_communication();       // Inizializzazione delle seriali
    init_sensors();             // Inizializzazione dei sensori

    delay(2000); // Attendi l'inizializzazione delle comunicazioni
}

uint8_t last_command;
uint8_t last_command_data;
void handle_command(){
    char debugMsg[128]; // Buffer per la stringa di debug
    sprintf(debugMsg, "Ricevuto Comando: %d con dati: %d", last_command, last_command_data);
    sendPacket(MSG_DEBUG, (uint8_t*)debugMsg, strlen(debugMsg));
}

unsigned long lastTelemetryTime;
void loop() {
    // Read and Send Telemetry to Rapsberry
    unsigned long now = millis();
    if (now - lastTelemetryTime > 1000) {

        get_gyro_data();

        if(read_command(&last_command, &last_command_data)){
            handle_command();
        }

        // Invia telemetria (binario)
        sendPacket(MSG_TELEMETRY, (uint8_t*)&currentTelemetry, sizeof(currentTelemetry));

        // Invia messaggio di debug (stringa ASCII, ma incapsulata nel pacchetto binario)
        const char* debugMsg = "Ho inviato Telemetria";
        sendPacket(MSG_DEBUG, (uint8_t*)debugMsg, strlen(debugMsg));

        lastTelemetryTime = now;
    }
}