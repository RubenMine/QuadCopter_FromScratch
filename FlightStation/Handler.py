import serial
import threading
import json
import time
import struct

class PacketHandler:
    """
    Modulo che si occupa di tenere tutte funzioni statiche per la gestione dei dati.
    """

    # PACKET CONSTANTS    
    START_BYTE = 0x7E
    END_BYTE = 0x7F

    # PACKET TYPES
    PKT_TYPE_TELEMETRY = 0x01
    PKT_TYPE_STATUS    = 0x02
    PKT_TYPE_DEBUG     = 0x03
    PKT_TYPE_COMMAND   = 0x04  
    PKT_TYPE_MOTOR   = 0x05  

    MAP_PACKET_TYPE = {
        PKT_TYPE_TELEMETRY: "TELEMETRY",
        PKT_TYPE_STATUS: "STATUS",
        PKT_TYPE_DEBUG: "DEBUG",
        PKT_TYPE_COMMAND: "COMMAND"
        PKT_TYPE_MOTOR:	"MOTOR" 
    }


    # COMMAND TYPES
    CMD_TYPE_SET   = 0x00
    CMD_TYPE_STATE = 0x01
    
    MAP_COMMAND_TYPE = {
        "SET": CMD_TYPE_SET,
        "STATE": CMD_TYPE_STATE
    }


    # STATE TYPES
    MAP_STATE_TYPE = {
        0: "WAIT",
        1: "READY",
        2: "FLYING"
    }

    @staticmethod
    def unpack(packet_bytes):
        """
        Il formato del pacchetto (packet_bytes) è:
          [tipo][len_low][len_high][payload...][checksum]
        Restituisce un dizionario con i dati oppure None in caso di errore.
        """
        if len(packet_bytes) < 4:
            print("Pacchetto troppo corto")
            return None
        packet_type = packet_bytes[0]
        length = packet_bytes[1] | (packet_bytes[2] << 8)
        if len(packet_bytes) != 3 + length + 1:
            print("Lunghezza non corrisponde: atteso", 3+length+1, "ricevuto", len(packet_bytes))
            return None
        payload = packet_bytes[3:3+length]
        received_checksum = packet_bytes[3+length]
        calc_checksum = (packet_type + packet_bytes[1] + packet_bytes[2] + sum(payload)) & 0xFF
        if calc_checksum != received_checksum:
            print("Checksum errato: calcolato", calc_checksum, "ricevuto", received_checksum)
            return None
        # Costruiamo il dizionario. Per le struct, qui ad esempio lasciamo il payload come array di byte.
        # In un caso reale potresti decidere di decodificare in base al tipo.
        data = {
            "pckt_type": PacketHandler.MAP_PACKET_TYPE.get(packet_type, "UNKNOWN"),
            "length": length,
            "payload": PacketHandler.parse_payload(packet_type, payload)
        }

        return data
    

    @staticmethod
    def parse_payload(packet_type, payload):
        if packet_type == PacketHandler.PKT_TYPE_TELEMETRY:
            return PacketHandler.unpack_telemetry(payload)
        
        elif packet_type == PacketHandler.PKT_TYPE_STATUS:
            return PacketHandler.unpack_status(payload)
        
        elif packet_type == PacketHandler.PKT_TYPE_DEBUG:
            return PacketHandler.unpack_debug(payload)
        
        elif packet_type == PacketHandler.PKT_TYPE_MOTOR:
            return PacketHandler.unpack_motor(payload)
            
        else:
            return None



    @staticmethod  
    def unpack_telemetry(payload):
        roll, pitch, yaw, altitude = struct.unpack('ffff', payload)
        unpacked_data = {
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "altitude": altitude
        }   
        return unpacked_data
    
    @staticmethod
    def unpack_status(payload):
        state, = struct.unpack('B', payload)
        unpacked_data = {
            "state": PacketHandler.MAP_STATE_TYPE[state]
        }
        return unpacked_data
    
    @staticmethod
    def unpack_motor(payload):
        pwm1, pwm2, pwm3, pwm4 = struct.unpack('dddd', payload)
        unpacked_data = {
            "M1": pwm1,
            "M2": pwm2,
            "M3": pwm3,
            "M4": pwm4
        }
        
        return unpacked_data
    
    def unpack_debug(payload):
        debug_mess = payload.decode('utf-8', errors='replace')
        unpacked_data = {
            "message": debug_mess
        }
        return unpacked_data
    
    

    @staticmethod
    def pack(packet_type, payload_bytes):
        """
        Impacchetta i dati (packet_type, payload_bytes) in un pacchetto
        conforme al protocollo:
          START | packet_type | len_low | len_high | payload ... | checksum | STOP
        """
        length = len(payload_bytes)
        # Calcolo checksum (uguale a come fatto in C):
        #  packet_type + (len_low) + (len_high) + sum(payload_bytes) (mod 256)
        checksum_calc = (
            packet_type
            + (length & 0xFF)
            + ((length >> 8) & 0xFF)
            + sum(payload_bytes)
        ) & 0xFF
        
        # Costruisci il pacchetto
        packet = bytearray()
        packet.append(PacketHandler.START_BYTE)
        packet.append(packet_type)
        packet.append(length & 0xFF)
        packet.append((length >> 8) & 0xFF)
        packet.extend(payload_bytes)
        packet.append(checksum_calc)
        packet.append(PacketHandler.END_BYTE)
        
        return bytes(packet)

    @staticmethod
    def create_command_payload(command: dict):
        """
        Prepara un pacchetto di 16 byte secondo il formato:
        Byte 1: command_type (0 -> SET, 1 -> STATE, ecc.)
        Byte 2..11: info string (10 byte)
        Byte 12..15: float (little-endian)
        """
        
        # Byte 0: start byte
        packet = bytearray()

        # Byte 0: command type
        cmd_type_str = command.get("command_type", "").upper()
        cmd_type = PacketHandler.MAP_COMMAND_TYPE.get(cmd_type_str, None)
        if cmd_type is None:
            return "Error: Unknown command type."
        packet.append(cmd_type)

        # Byte 1..10: info string (10 bytes)
        info_str = command.get("command_info", "")
        info_bytes = info_str.encode('ascii', 'replace')[:10]
        info_bytes += b'\x00' * (10 - len(info_bytes))
        packet.extend(info_bytes)

        # Byte 11..14: float value (little-endian)
        value = command.get("command_value", 0.0)
        value_bytes = struct.pack('<f', value)
        packet.extend(value_bytes)

        return bytes(packet)
    
    
    

