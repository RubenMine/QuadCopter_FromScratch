class Handler():
    _instance = None  # Memorizza l'istanza singleton

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialize(*args, **kwargs)
        return cls._instance

    def _initialize(self):
        """implementare di classe in classe"""
        pass



from CommProtocols import UARTAdapter, HTTPAdapter

class CommunicationHandler():
    def __init__(self):
        # Dizionario di adapter per i vari protocolli
        self.adapters = {
            'UART': UARTAdapter(),
            'HTTP': HTTPAdapter()
            #'LoRa': LoRaAdapter()
        }

    def send(self, protocol, data, **kwargs):
        """
        Invoca il metodo di invio del protocollo corretto.
        """
        if protocol in self.adapters:
            return self.adapters[protocol].send(data, **kwargs)
        else:
            raise ValueError(f"Unsupported protocol: {protocol}")

    def receive(self, protocol, **kwargs):
        """
        Invoca il metodo di ricezione del protocollo corretto.
        """
        if protocol in self.adapters:
            return self.adapters[protocol].receive(**kwargs)
            #return DataHandler().format(data)
        else:
            raise ValueError(f"Unsupported protocol: {protocol}")
        
    def read(self, protocol, **kwargs):
        """
        Invoca il metodo di ricezione del protocollo corretto.
        """
        if protocol in self.adapters:
            return self.adapters[protocol].read(**kwargs)
            #return DataHandler().format(data)
        else:
            raise ValueError(f"Unsupported protocol: {protocol}")


import struct

class DataHandler(Handler):

    switcher = {
            0: "WAIT",
            1: "READY",
            2: "FLIGHT"
    }

    def unpack_telemetry(self, data):
        unpacked_data = dict()

        roll, pitch, yaw, altitude = struct.unpack('ffff', data)
        unpacked_data = {
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "altitude": altitude
            }
           
        return unpacked_data

 
    def unpack_status(self, data):
        unpacked_data = dict()

        state, = struct.unpack('B', data)
        print(state)

        unpacked_data = {
                "state": self.switcher[state]
            }
        
        return unpacked_data