// Packet.h
#ifndef PACKET_H
#define PACKET_H

#include <Arduino.h>
#include <avr/pgmspace.h>

//---------------------------------------------------------------------
// COSTANTI DEL NUOVO PROTOCOLLO
//---------------------------------------------------------------------
#define PACKET_START 0x7E
#define PACKET_STOP  0x7F

// Tipi di pacchetto
enum PacketType {
    PKT_TYPE_TELEMETRY = 0x01,
    PKT_TYPE_STATUS    = 0x02,
    PKT_TYPE_DEBUG     = 0x03,
    PKT_TYPE_COMMAND   = 0x04,
    PKT_TYPE_MOTOR     = 0x05
};

// Prototipi delle funzioni
uint8_t computeChecksum(uint8_t type, uint16_t len, const uint8_t* payload);
void sendPacket(uint8_t type, const uint8_t* payload, uint16_t len);

void sendTelemetry();
void sendStatus(FlightState state);
void sendDebug(const char* msg);
void sendMotor();

//void sendCommandPacketPythonFormat(uint8_t cmd_type, const char* info, float value);

#endif

