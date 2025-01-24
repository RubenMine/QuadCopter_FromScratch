# server.py
from flask import Flask, request, jsonify
from Handlers import CommunicationHandler


app = Flask(__name__)


import struct

def pack_command(cmd_type_str, cmd_info_str, cmd_value_float):
    """
    Prepara un pacchetto di 16 byte secondo il formato:
      Byte 0: 0xAA
      Byte 1: command_type (0 -> SET, 1 -> STATE, ecc.)
      Byte 2..11: info string (10 byte)
      Byte 12..15: float (little-endian)
    """
    # Mappa la stringa del command_type nel valore numerico previsto
    if cmd_type_str.upper() == "SET":
        cmd_type_byte = 0
    elif cmd_type_str.upper() == "STATE":
        cmd_type_byte = 1
    else:
        # Eventuale default o gestione di errori
        cmd_type_byte = 255

    # Creiamo un array di 16 byte
    packet = bytearray(16)

    # Byte 0: start byte
    packet[0] = 0xCC

    # Byte 1: command type
    packet[1] = cmd_type_byte

    # Byte 2..11: info string (10 byte)
    # Se la stringa è più lunga di 10, la tagliamo; se è più corta, la pad con 0x00
    encoded_info = cmd_info_str.encode('ascii', 'replace')[:10]
    for i in range(10):
        packet[2 + i] = encoded_info[i] if i < len(encoded_info) else 0x00

    # Byte 12..15: float in formato little-endian
    float_bytes = struct.pack('<f', cmd_value_float)
    packet[12] = float_bytes[0]
    packet[13] = float_bytes[1]
    packet[14] = float_bytes[2]
    packet[15] = float_bytes[3]

    return bytes(packet)




@app.route('/command', methods=['POST'])
def receive_command():
    comm = CommunicationHandler()
    data = request.get_json()

    command_type = data.get('command_type', None)
    command_info = data.get('command_info', None)
    command_value = data.get('command_value', None)

    print("=== Comando Ricevuto ===")
    print(f"Tipo:  {command_type}")
    print(f"Info:  {command_info}")
    print(f"Valore: {command_value}")
    print("========================")
 
    # Invia il comando al raspberry pi
    pack = pack_command(command_type, command_info, float(command_value))
    print(pack)
    comm.send("UART", pack)


    # Risposta al client
    return jsonify({"status": "ok", "message": "Comando ricevuto"}), 200



from Modules import CommunicationModule

if __name__ == '__main__':
    # Esegui il server in ascolto su tutte le interfacce
    # e sulla porta 5000
    CommunicationModule().start()
    app.run(host='0.0.0.0', port=5001)
