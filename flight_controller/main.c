#include <Arduino.h>
#include <string.h> // Per strcmp
#include <stdint.h> // Per uint8_t, uint32_t
#include <stdbool.h> // Per il tipo bool


#include "flight_fsm.c"
#include "uart_comm.c"
#include "sensors.c"
#include "motor.c"
#include "pid_controller.c"
#include "command.c"

void setup() {}


void loop() {
    flight_fsm_run();
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

/*
// Read and Send Telemetry to Rapsberry
unsigned long now = millis();
if (now - lastTelemetryTime > 1000) {
    get_telemetry();
    drone_control();
    drone_motor();
    send_telemetry();
    
    if(read_command()){
        handle_command();
    }
    lastTelemetryTime = now;
}
*/