#include <stdint.h>
#include <stddef.h>


#define START_BYTE 10 //0xAA

#define MSG_DEBUG 1
#define MSG_TELEMETRY 2
#define UART_TIMEOUT 1000


void init_communication(){
    Serial.begin(9600);
}

// Funzione per calcolare il CRC8
uint8_t computeCRC8(const uint8_t *data, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; i++) {
        uint8_t inbyte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}


// Funzione generica per inviare un pacchetto
void sendPacket(uint8_t type, const uint8_t* data, uint8_t length) {
    uint8_t packet[3 + length + 1];
    packet[0] = START_BYTE;
    packet[1] = type;
    packet[2] = length;
    memcpy(&packet[3], data, length);

    uint8_t crc = computeCRC8(&packet[1], 1 + 1 + length);
    packet[3 + length] = crc;

    // Invia il pacchetto binario sulla seriale principale (USB)
    Serial.write(packet, sizeof(packet));
}

bool uart_read_byte(uint8_t *byte, uint32_t timeout);
bool read_command(uint8_t *command_type, uint8_t *command_data) {
    uint8_t byte;
    uint8_t crc_received, crc_calculated;

    // Leggi il primo byte e verifica che sia START_BYTE
    if (!uart_read_byte(&byte, UART_TIMEOUT) || byte != START_BYTE) {
        return false; // Pacchetto non valido
    }

    // Leggi COMMAND_TYPE
    if (!uart_read_byte(&byte, UART_TIMEOUT)) {
        return false; // Errore nella lettura
    }
    *command_type = byte;

    // Leggi DATA
    if (!uart_read_byte(&byte, UART_TIMEOUT)) {
        return false; // Errore nella lettura
    }
    *command_data = byte;

    // Leggi CRC
    if (!uart_read_byte(&crc_received, UART_TIMEOUT)) {
        return false; // Errore nella lettura
    }

    // Calcola il CRC del pacchetto
    //crc_calculated = calculate_crc(*command_type, *data);

    // Verifica che il CRC corrisponda
    //if (crc_calculated != crc_received) {
    //    return false; // CRC non corrisponde
    //}
    
    return true;
}


bool uart_read_byte(uint8_t *byte, uint32_t timeout) {
    unsigned long start_time = millis(); // Ottieni il tempo attuale in millisecondi

    // Aspetta fino a quando i dati sono disponibili o il timeout scade
    while (!Serial.available()) {
        if (millis() - start_time >= timeout) {
            return false; // Timeout scaduto, nessun dato ricevuto
        }
    }

    // Legge un byte dai dati disponibili sulla UART
    *byte = Serial.read();
    return true;
}
