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

class CommunicationHandler(Handler):
    def _initialize(self):
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

    def read(self, protocol, **kwargs):
        """
        Invoca il metodo di ricezione del protocollo corretto.
        """
        if protocol in self.adapters:
            return self.adapters[protocol].receive(**kwargs)
            #return DataHandler().format(data)
        else:
            raise ValueError(f"Unsupported protocol: {protocol}")


import struct

class DataHandler(Handler):
    def _initialize(self):
        # Define packet type
        self.TELEMETRY = 1
        self.CONTROLLER_DEBUG = 2
        self.PROCESSOR_DEBUG = 3
        self.COMMAND = 4
        self.TAKEOFF = 5
        pass

    def format_packet_into_json(self, type, data):
        f_data = dict()

        if type == self.TELEMETRY:
            roll, pitch, yaw, altitude = struct.unpack('ffff', data)
            f_data["type"] = "telemetry"
            f_data["data"] = {
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "altitude": altitude
            }

        elif type == self.CONTROLLER_DEBUG:
            debug_str = data.decode('utf-8', errors='replace')
            f_data["type"] = "controller_debug"
            f_data["data"] = {
                "msg": debug_str
            }

        elif type == self.COMMAND:
            ctype, cdata = struct.unpack('dd', data)
            
            f_data["type"] = "command"
            if ctype == self.TAKEOFF:
                f_data["data"] = {
                "command_type": "takeoff",
                "command_data": cdata
            }

        else:
            f_data = {
                'type': 'unknown',
                'data': data
            }

        return f_data

    def format_json_into_packet(data):
        type = data["type"]

        if type == "command":
            pass
        