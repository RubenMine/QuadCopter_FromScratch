// Commands.h
#ifndef COMMANDS_H
#define COMMANDS_H

#include <Arduino.h>

// Tipi di comandi
enum CommandType {
    SET = 0,
    STATE = 1
};

// Struttura per i comandi
struct Command {
    CommandType type;  // Byte0
    char info[11];             // Byte1..10
    float value;               // Byte11..14
};

extern Command cmd;


void processCommandPacket();
void processIncomingSerial();
void pack_command();


#endif
