// Packet.cpp
#include "packet.h"
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>


uint8_t computeChecksum(uint8_t type, uint16_t len, const uint8_t* payload) {
    uint16_t sum = type;
    sum += (len & 0xFF);
    sum += ((len >> 8) & 0xFF);
    for (uint16_t i = 0; i < len; i++) {
        sum += payload[i];
    }
    return (uint8_t)(sum & 0xFF);
}

void sendPacket(uint8_t type, const uint8_t* payload, uint16_t len) {
    Serial.write(PACKET_START);
    Serial.write(type);
    Serial.write((uint8_t)(len & 0xFF));
    Serial.write((uint8_t)((len >> 8) & 0xFF));
    for (uint16_t i = 0; i < len; i++) {
        Serial.write(payload[i]);
    }
    uint8_t checksum = computeChecksum(type, len, payload);
    Serial.write(checksum);
    Serial.write(PACKET_STOP);
}


void sendDebug(const __FlashStringHelper* message) {
  char buffer[128];
  strncpy_P(buffer, (PGM_P)message, sizeof(buffer));
  buffer[sizeof(buffer) - 1] = '\0';
  sendDebug(buffer);
}

void sendDebug(const char* message) {
    uint16_t len = strlen(message);
    sendPacket(PKT_TYPE_DEBUG, (const uint8_t*)message, len);
}


void sendTelemetry() {
    // Invia 16 byte: roll, pitch, yaw, altitude (4 float da 4 byte)
    uint8_t data[16];
    memcpy(&data[0],  &currentTelemetryData.roll, 4);
    memcpy(&data[4],  &currentTelemetryData.pitch, 4);
    memcpy(&data[8],  &currentTelemetryData.yaw, 4);
    memcpy(&data[12], &currentTelemetryData.altitude, 4);
    sendPacket(PKT_TYPE_TELEMETRY, data, 16);
}

void sendStatus(FlightState state) {
    char msg[40];
    snprintf(msg, sizeof(msg), "sendStatus: Nuovo stato=%d", state);
    //sendDebug(msg);
    uint8_t data = (uint8_t)state;
    sendPacket(PKT_TYPE_STATUS, &data, 1);
}


void sendMotor(){
  // Invia 16 byte: roll, pitch, yaw, altitude (4 float da 4 byte)
    uint8_t data[16];
    memcpy(&data[0],  &pw.motor_pwm[0], sizeof(int));
    memcpy(&data[4],  &pw.motor_pwm[1], sizeof(int));
    memcpy(&data[8],  &pw.motor_pwm[2], sizeof(int));
    memcpy(&data[12], &pw.motor_pwm[3], sizeof(int));
    sendPacket(PKT_TYPE_MOTOR, data, 16);
}
