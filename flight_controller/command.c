#include "header/command.h"
#include "header/motor.h"


char* handle_command(){

    switch (command.command_type) {
        case SET:
            set_value_of(command.info, command.value);
            break;

        case STATE:
            flight_fsm_transition(command.info);

        default:
            printf("[DEBUG] Received unknown command type!\n");
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
            command.state = (enum CommandState)buffer[1];

            // Copia i 10 byte successivi nella stringa e aggiunge il terminatore
            memcpy(command.string, &buffer[2], 10);
            command.string[9] = '\0'; // Assicura la terminazione della stringa

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
        printf("[DEBUG] Received unknown info type!\n");
    }
}