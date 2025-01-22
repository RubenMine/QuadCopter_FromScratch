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



