import struct
from Communications import HTTPConnection, SerialReader
from Handler import PacketHandler
import threading


class CommunicationModule():
    """
    Modulo principale che gestisce la comunicazione tra: 
        Controller <--UART--> FlightStation <--HTTP--> GroundStation
        
    """

    def __init__(self):
        self.GroundConnection = HTTPConnection(
                                               server_url="http://192.168.1.197:6001",          
                                               local_url="http://0.0.0.0:5001",                  
                                               callback=self.on_http_data_received
                                              )
        
        self.controller = SerialReader(
                                        port="/dev/ttyACM0",                                
                                        baudrate=115200,
                                        callback=self.on_uart_data_received   
                                      )
    
    def run(self):
        self.GroundConnection.start()
        self.controller.start()
        print("[CommunicationModule] HTTPGroundConnection avviato.")


    def on_http_data_received(self, data):
        print(f"[CommunicationModule] Dati ricevuti via HTTP: {data}")
        # Inoltra i dati al Controller
        cmd = PacketHandler.create_command_payload(data)
        pck = PacketHandler.pack(PacketHandler.PKT_TYPE_COMMAND, cmd)
        
        print("COMANDO E PACCHETTO ASSOCIATO")
        print(cmd)
        print(pck)
        """
        threading.Thread(
            target=self.controller.send_packet,
            args=(pck),
            daemon=True
        ).start()
        """
        self.controller.send_packet(pck)

        #import time
        #time.sleep(0.1)


    def on_uart_data_received(self, data):
        #print(f"[CommunicationModule] Dati ricevuti via UART: {data}")
        
        #print(data)
        if data["pckt_type"] != "DEBUG":
            # Inoltra i dati al GroundStation
            threading.Thread(
            target=self.GroundConnection.send_data,
            args=(data["pckt_type"], data["payload"]),
            daemon=True
            ).start()
            #self.GroundConnection.send_data(data["pckt_type"], data["payload"])
        if 
        else:
            print(data)
            pass



if __name__ == "__main__":
    CommunicationModule().run()    

    while(True):
        pass
