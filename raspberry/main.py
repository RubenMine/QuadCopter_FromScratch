from MissionStateMachine import MissionCoordinator
from Modules import CommunicationModule

modules = [CommunicationModule()]
sfm = MissionCoordinator(modules)
sfm.run()
