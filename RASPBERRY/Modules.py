from threading import Thread
#from MissionStateMachine import MissionCoordinator, DroneState
import struct

class Module(Thread):
    def __init__(self):
        super().__init__()
        #self.current_state = state

        
    def update_state(self, new_state):
        self.current_state = new_state

    def run(self):
        # Add the code to be executed when the thread runs
        pass



from Handlers import CommunicationHandler, DataHandler
from time import sleep

class CommunicationModule(Module):
    """Gestione delle comunicazioni tra UART (flight controller) e Home Assistant (HTTP)."""
    def __init__(self):
        super().__init__()
        self.comm = CommunicationHandler()
    
    def run(self):
        while(True):
            read_data = self.comm.read("UART")

            if self.isPacket(read_data, "telemetry"):
                print(read_data)
                telemetry = DataHandler().unpack_telemetry(read_data[1:17])
                self.comm.send("HTTP", telemetry, endpoint="/telemetry")    
                           	
            elif self.isPacket(read_data, "status"):
                #print(read_data[1:2])
                status = DataHandler().unpack_status(read_data[1:2])
                self.comm.send("HTTP", status, endpoint="/status"); 
            
            elif self.isPacket(read_data, "command"):
                print("Command Packet")

            else:
                print(read_data)

            #sleep(0.5)

    def isPacket(self, data, type):
        if data and data[0] == 0xAA and type == "telemetry" and len(data) == 20:
            return True
        elif data and data[0] == 0xBB and type == "status":
            return True
        elif data and data[0] == 0xCC and type == "command":
            return True
        else:
            return False

"""
            if self.current_state == DroneState.PREFLIGHT:
                try:
                    self.comm = CommunicationHandler()
                except Exception as e:
                    print(f"Errore Inizializzazione CommunicationModule: {e}")
                    raise e

                self.send_event("check_complete") # SEND EVENT ONLY SEND REAL EVENT NOT STRING (TO CHANGE)

                print("Verifica connessione UART...")
                CommunicationHandler.send("UART", "Check_Commuication_CMD")

                data_type = None
                while(data_type == None): 
                    data_type, payload = CommunicationHandler.read("UART")    
                print("Connessione UART OK")

                print("Connessione Home Assistant...")
                try:
                    CommunicationHandler.send_data(CommType.HTTP, DataType.COMMAND, "START")
                    response = CommunicationHandler.read_data(CommType.HTTP)
                except Exception as e:
                    print(f"Errore Home Assistant: {e}")
                

            elif self.current_state == DroneState.READY:
                Ciclo principale di comunicazione
                self.handle_controller_comm()
                #self.handle_command_comm()
                #time.sleep(0.5)  # per evitare loop troppo stretti
"""
   	 
"""
        #data_type, payload = CommunicationHandler().read("HTTP")
        #if data_type != None:
        #    if data_type == "cmd":
        #        #if( CommandHandler.verify(payload) ):
        #            CommunicationHandler().send("UART", data_type, payload)
        #
        #data_type, payload = CommunicationHandler().read("LoRa")
        #if data_type != None:
        #        if data_type == "cmd":
        #            #if( CommandHandler.verify(payload) ):
        #                CommunicationHandler().send("UART", data_type, payload)
"""


"""        
    def handle_controller_comm(self):
        data = CommunicationHandler().read("UART")

        if data["type"] == "telemetry":
            CommunicationHandler().send("HTTP", data["data"])

        elif data["type"] == "controller_debug":
            #LoggerHandler.write(payload)
            pass
    
        
"""
"""
class CommandModule(Module):

    def run(self):
        while True:
            # Legge una riga di input da terminale
            user_input = input("Inserisci Comando [Target-CC-ID-Value]>  ")
            command = user_input.strip().split()

            if command[0] == "rasp":
                
            elif command[0] == "controller":
            
"""


