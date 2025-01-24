from serial import Serial
import struct
import time
#from Handlers import DataHandler

# Costanti di protocollo
START_BYTE = 10

@staticmethod
def compute_crc8(data: bytes) -> int:
    """
    Stessa logica di calcolo CRC8 che hai nel codice Arduino:
    data: sequenza di byte (type + length + payload) su cui calcolare il CRC.
    """
    crc = 0x00
    for b in data:
        inbyte = b
        for _ in range(8):
            mix = (crc ^ inbyte) & 0x01
            crc >>= 1
            if mix:
                crc ^= 0x8C
            inbyte >>= 1
    return crc

class UARTAdapter:
    def __init__(self, port='/dev/ttyACM0', baudrate=9600):
        self.serial_port = Serial(port, baudrate, timeout=1)

    def send(self, data, **kwargs):
        """
        Invia dati al microcontrollore via UART.
        Se data è una stringa, la converte in bytes UTF-8.
        """
        #packet = DataHandler().format_json_into_packet(data)
        self.serial_port.write(data)
        return f"UART: Sent {data}"

	
    def read(self):
        """read raw line from serial port"""
        return self.serial_port.readline()

"""
    def receive(self):
        
        Legge un pacchetto binario e lo decodifica in base a msg_type.
        Ritorna un dizionario con le informazioni decodificate, ad esempio:
          {
            'type': 'debug',
            'data': 'Stringa di debug'
          }
        o
          {
            'type': 'telemetry',
            'data': {
                'roll': 12.34,
                'pitch': 56.78,
                'yaw': 90.12,
                'altitude': 100.5
            } 
          }
        Se nessun pacchetto è disponibile o ci sono errori, restituisce None.
        
        packet = self.receive_packet()

        if packet is None:
            return None

        msg_type, payload = packet
        DataHandler().format_packet_into_json(msg_type, payload)



    #read_packet
    def receive_packet(self):
        
        Legge un pacchetto dal flusso seriale seguendo il formato:
          [START_BYTE | TYPE | LENGTH | DATA... | CRC]

        Restituisce (msg_type, payload) se tutto OK, altrimenti None.

        # Legge 1 byte
        start = self.serial_port.read(1)
        if len(start) == 0:
            # Timeout o nessun dato
            return None
        if start[0] == START_BYTE:
            # Leggi TYPE
            t = self.serial_port.read(1)
            if len(t) < 1:
                return None
            msg_type = t[0]
            # Leggi LENGTH
            l = self.serial_port.read(1)
            if len(l) < 1:
                return None
            length = l[0]
            # Leggi i 'length' byte di payload
            payload = self.serial_port.read(length)
            if len(payload) < length:
                return None
            # Leggi CRC
            crc_byte = self.serial_port.read(1)
            if len(crc_byte) < 1:
                return None
            crc_recv = crc_byte[0]
            # Calcola il CRC su (type + length + payload)
            data_for_crc = bytes([msg_type, length]) + payload
            crc_calc = compute_crc8(data_for_crc)
            if crc_calc == crc_recv:
                # Pacchetto valido
                return (msg_type, payload)
            else:
                # CRC sbagliato, scarta e continua a cercare START_BYTE
                print(f"CRC mismatch: ricevuto={crc_recv}, calcolato={crc_calc}")
                # In caso si desideri una gestione più robusta,
                # bisognerebbe cercare nuovamente START_BYTE all'interno di 'payload'.
                # Se non è START_BYTE, ricominciamo il ciclo e aspettiamo il prossimo byte. 

"""

import requests

class HTTPAdapter:
    def __init__(self, base_url='http://127.0.0.1:6000'):
        """
        base_url: L'endpoint del "client" a cui inviare richieste.
                  Di default ipotizziamo che il client sia in ascolto
                  su http://127.0.0.1:6000 (porta 6000).
        """
        self.base_url = base_url

    def send(self, data, endpoint='/telemetry', **kwargs):
        """
        Esegue una POST verso: base_url + endpoint
        Invia 'data' in formato JSON.
        Esempio:
            adapter = HTTPAdapter('http://192.168.1.50:6000')
            adapter.send({"key": "val"}, '/telemetry')
        """
        url = f"{self.base_url}{endpoint}"
        try:
            resp = requests.post(url, json=data, **kwargs)
            return resp
        except Exception as e:
            print(f"[HTTPAdapter] Errore inviando a {url}: {e}")
            return None

    def receive(self, endpoint='/api/config', **kwargs):
        """
        Esegue una GET verso: base_url + endpoint
        Ritorna il JSON (dict) se status_code=200, altrimenti None.
        Esempio:
            adapter = HTTPAdapter('http://192.168.1.50:6000')
            config_data = adapter.receive('/api/config')
        """
        url = f"{self.base_url}{endpoint}"
        try:
            resp = requests.get(url, **kwargs)
            if resp.status_code == 200:
                return resp.json()
            else:
                print(f"[HTTPAdapter] Errore GET {url}: {resp.status_code}")
                return None
        except Exception as e:
            print(f"[HTTPAdapter] Eccezione GET {url}: {e}")
            return None
