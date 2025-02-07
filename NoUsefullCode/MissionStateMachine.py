from enum import Enum

#from Module import Module


class DroneState(Enum):
    PREFLIGHT = "Preflight Check"
    READY = "Ready"
    MISSION1 = "Mission1"


class MissionCoordinator:
    _instance = None  # Variabile di classe per memorizzare l'istanza

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, modules = None):
        if not hasattr(self, "initialized"):  # Evita la reinizializzazione
            self.modules = modules
            self.initialized = True
            self.change_state(DroneState.PREFLIGHT)

    def handle_event(self, module_name, event):
        print(f"Event from {module_name}: {event}")
        self.events.append(event)

        if self.state == DroneState.PREFLIGHT and "check_complete" in self.events:
            self.change_state(DroneState.READY)

        elif self.state == DroneState.READY and "takeoff" in self.events:
            self.change_state(DroneState.MISSION1)
            

    def change_state(self, state: DroneState):
        self.state = state
        for module in self.modules:
            module.update_state(state)
        self.events = []
        print(f"State changed to {state}.")

    def run(self):
        # start all modules thread
        for m in self.modules:
            m.start()

    def notify_module(self, target_module, message):
        target_module.receive_message(message)
