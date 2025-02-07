#include "command.h"
#include "motor.h"


static Command cmd;

#define RX_BUFFER_SIZE 128
uint8_t rxBuffer[RX_BUFFER_SIZE];
uint8_t rxIndex = 0;
bool receivingPacket = false;


void pack_command(){
    cmd.type = (CommandType)rxBuffer[3];
    
    memcpy(cmd.info, 3 + rxBuffer + 1, 10);
    cmd.info[10] = '\0';

    memcpy((void*)&cmd.value, 3 + rxBuffer + 11, sizeof(float));
}


extern interpol_values v;
extern PID altitude_pid;

void processCommandPacket(){
    pack_command();

    switch (cmd.type) {
    
        case SET:
            
            /// SET VARIABLE VALUE ///
            if (strcmp(cmd.info, "ALTITUDE") == 0){
              v.old_setpoint= altitude_pid.setpoint;
              v.new_setpoint= cmd.value;

              sendDebug("ALTITUDE CHANGE");
            	//settings.altitude_setpoints.old_setpoint = ?
            	//settings.altitude_setpoints.new_setpoint = cmd.value;
            }
            
            else if (strcmp(cmd.info, "THROTTLE") == 0){
              sendDebug("THROTTLE CHANGE");
            	setting.throttle = cmd.value/100;
            }
            
            else if (strcmp(cmd.info, "MAXPWM") == 0){
            	setting.maxPWM = cmd.value;
            }
            break;

        case STATE:
            sendDebug("COMANDO STATE ARRIVATO");
            sendDebug(cmd.info);
            /// STATE TRANSITION ///
            flight_fsm_transition(cmd.info);

        default:
            //sendDebug("[DEBUG] Received unknown command type!\n");
            break;
    }

    return;
}


void processIncomingSerial() {
    if (!Serial) {
        flight_fsm_transition("STOP");
    }

    while (Serial.available() > 0) {
        uint8_t byteIn = Serial.read();
       
        if (byteIn == PACKET_START) {
            receivingPacket = true;
            rxIndex = 0;
            //sendDebug(F("[DEBUG] processIncomingSerial() - Inizio pacchetto"));
            continue;
        }
        
        if (receivingPacket) {
            if (byteIn == PACKET_STOP) {

                if (rxIndex < 4) { 
                    //sendDebug(F("processIncomingSerial: Pacchetto troppo corto"));
                    receivingPacket = false;
                    return;
                }
                
                uint8_t pktType = rxBuffer[0];
                
                uint16_t lenPayload = rxBuffer[1] | (rxBuffer[2] << 8);
                if (rxIndex != (3 + lenPayload + 1)) { 
                    //sendDebug(F("processIncomingSerial: Lunghezza pacchetto non valida"));
                    receivingPacket = false;
                    return;
                }
                
                uint8_t receivedChecksum = rxBuffer[3 + lenPayload];
                uint8_t calcChecksum = computeChecksum(pktType, lenPayload, rxBuffer + 3);
                if (calcChecksum != receivedChecksum) {
                    //sendDebug(F("Cmd: Checksum errato"));
                    receivingPacket = false;
                    return;
                }
                
        	if (pktType == PKT_TYPE_COMMAND)
                    processCommandPacket();

        	receivingPacket = false;
        	
            } else {
            
                if (rxIndex < RX_BUFFER_SIZE)
                    rxBuffer[rxIndex++] = byteIn;
                else {
                    //sendDebug(F("processIncomingSerial: Buffer overflow"));
                    receivingPacket = false;
                    rxIndex = 0;
                }
            }
        }
    }
}
