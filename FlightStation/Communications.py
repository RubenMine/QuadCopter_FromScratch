import threading
import time

class Listener():
    def __init__(self, callback= None, thread_func= None):
        """
        :param parent_module: riferimento al modulo padre che contiene i callbacks.
        """

        self.callback = callback

        self.thread_func = thread_func
        self.thread = None

    def start(self):
        if self.thread_func:
            self.thread = threading.Thread(target=self.thread_func, daemon=True)
            self.thread.start()

    def stop(self):
        self.thread.join()


from Handler import PacketHandler
import serial

class SerialReader(Listener):
    def __init__(self, port= "/dev/ACM0", baudrate= 115200, callback= None):
        super().__init__(callback, self.start_reading)
        self.ser = serial.Serial(port, baudrate, timeout=0.1, rtscts=True)
        self.buffer = bytearray()
        self.running = True


    def start_reading(self):
        while self.running:

            if self.ser.in_waiting:
                byte_read = self.ser.read(1)
                if byte_read:
                    b = byte_read[0]

                    if b == PacketHandler.START_BYTE:
                        # Iniziamo un nuovo pacchetto
                        self.buffer = bytearray()

                    elif b == PacketHandler.END_BYTE:
                        # Fine pacchetto: processa il buffer
                        packet = bytes(self.buffer)
                        data = PacketHandler.unpack(packet)
                        #print(data)
                        if data:
                            self.callback(data)
                        self.buffer = bytearray()
                    else:
                        self.buffer.append(b)

            else:
                # Piccola pausa per evitare busy waiting
                time.sleep(0.001)

    def stop(self):
        super().stop()
        self.running = False
        self.ser.close()

    def send_packet(self, packet):
        """
        Impacchetta e invia i dati (payload_bytes) con packet_type sulla stessa UART
        usando il medesimo protocollo (START, lunghezza, checksum, STOP).
        """
        self.ser.write(packet)



from flask import Flask, request, jsonify
import requests

class HTTPConnection(Listener):
    """
    Classe che funge da client (invio dati e registrazione) e server Flask (ricezione comandi).
    """

    def __init__(self, server_url="http://<IP_GROUND>:<PORT_GROUND>", local_url="http://0.0.0.0:5001", callback= None):
        """
        :param server_url: URL della GroundStation a cui mandare l'IP locale e i dati.
        :param local_url: URL/porta su cui questa macchina (veicolo) avvia Flask.
        """
        super().__init__(callback, self.run_server)
        self.server_url = server_url
        self.local_url = local_url

        self.app = Flask(__name__)
        self.app.add_url_rule('/incoming_data', 'incoming_data', self._receive_data, methods=['POST'])

        self.server_thread = None


    def start(self):
        """
        1. Avvia il server Flask (in un thread separato).
        2. Tenta la registrazione presso la GroundStation in un ciclo retry.
        """
        super().start()
        #self._register_with_server_retry()


    def run_server(self):
        try:
            port = int(self.local_url.split(':')[-1])
            self.app.run(host='0.0.0.0', port=port, debug=False)
        except Exception as e:
            print(f"[HTTPGroundConnection] Errore durante l'avvio del server: {e}")
            return


    def _register_with_server_retry(self, retry_interval=3):
        """Prova a registrarsi al Server finché non ottieni 200 OK."""

        while True:
            try:
                payload = {"ip": self.local_url}
                
                response = requests.post(f"{self.server_url}/register", json=payload, timeout=5)

                if response.status_code == 200:
                    print(f"[HTTPGroundConnection] Registrazione avvenuta con successo: {response.json()}")
                    break
                else:
                    print(f"[HTTPGroundConnection] Errore di registrazione: {response.status_code} - {response.text}")
            
            except Exception as e:
                print(f"[HTTPGroundConnection] Connessione fallita, ritento tra {retry_interval} secondi... ({e})")
            
            time.sleep(retry_interval)


    def send_data(self, path, data):
        """Invia dati generici al server remoto."""
        try:
            #print(f"[HTTPGroundConnection] Invio dati a {path}: {data}")
            response = requests.post(f"{self.server_url}/{path}", json=data)
            #response = requests.post(f"http://192.168.1.197:6001/STATUS", json=data)
            if response.status_code == 200:
                #print(f"[HTTPGroundConnection] Dati inviati con successo a {path}: {response.json()}")
                pass
            else:
                print(f"[HTTPGroundConnection] Errore durante l'invio dei dati a {path}: {response.status_code} - {response.text}")
        except Exception as e:
            print(f"[HTTPGroundConnection] Errore durante l'invio dei dati a {path}: {e}")


    def _receive_data(self):
            """Route Flask per ricevere dati dal server."""
            data = request.get_json()
            if data:
                self.callback(data)
            return "OK", 200

