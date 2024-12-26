from threading import Thread
from MissionStateMachine import MissionCoordinator, DroneState
import struct

class Module(Thread):
    def __init__(self, state = DroneState.PREFLIGHT):
        super().__init__()
        self.current_state = state

    def send_event(self, str):
        MissionCoordinator().handle_event(self, str)
        
    def update_state(self, new_state):
        self.current_state = new_state

    def run(self):
        # Add the code to be executed when the thread runs
        pass



from Handlers import CommunicationHandler, DataHandler

class CommunicationModule(Module):
    """Gestione delle comunicazioni tra UART (flight controller) e Home Assistant (HTTP)."""
    
    def run(self):
        while(True):
            if self.current_state == DroneState.PREFLIGHT:
                try:
                    self.comm = CommunicationHandler()
                except Exception as e:
                    print(f"Errore Inizializzazione CommunicationModule: {e}")
                    raise e

                self.send_event("check_complete") # SEND EVENT ONLY SEND REAL EVENT NOT STRING (TO CHANGE)

                """
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
                """

            elif self.current_state == DroneState.READY:
                """Ciclo principale di comunicazione."""
                self.handle_controller_comm()
                self.handle_command_comm()
                #time.sleep(0.5)  # per evitare loop troppo stretti



    def handle_command_comm(self):
        data = CommunicationHandler().read("HTTP")
        #data = CommunicationHandler().read("LoRa")
        print(f"Data Received from HTTP/LORA: {data}")
        
        """
        data_type, payload = CommunicationHandler().read("HTTP")
        if data_type != None:
            if data_type == "cmd":
                #if( CommandHandler.verify(payload) ):
                    CommunicationHandler().send("UART", data_type, payload)
        
        data_type, payload = CommunicationHandler().read("LoRa")
        if data_type != None:
                if data_type == "cmd":
                    #if( CommandHandler.verify(payload) ):
                        CommunicationHandler().send("UART", data_type, payload)
        """
        
    def handle_controller_comm(self):
        data = CommunicationHandler().read("UART")

        if data["type"] == "telemetry":
            CommunicationHandler().send("HTTP", data["data"])

        elif data["type"] == "controller_debug":
            #LoggerHandler.write(payload)
            pass
    
        