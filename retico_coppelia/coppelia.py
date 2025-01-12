from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import retico_core


class JointForceIU(retico_core.abstract.IncrementalUnit):

    @staticmethod
    def type():
        return "JointForceIU"

    def __init__(self, creator=None, iuid=0, previous_iu=None, grounded_in=None, payload: dict[str: list]=None, **kwargs):
        super().__init__(creator=creator, iuid=iuid, previous_iu=previous_iu, grounded_in=grounded_in, payload=payload)
        if payload is not None:
            self.payload = payload
        else:
            self.payload = {}

    def set_single_force(self, joint_path, force):
        self.payload[joint_path] = force

    def set_multi_force(self, list_of_joint_paths, list_of_forces):
        for path, force in list(zip(list_of_joint_paths, list_of_forces)):
            self.payload[path] = force

    def __str__(self):
        return f"(JointForceIU: {self.payload.items()})"


class JointVelocityIU(retico_core.abstract.IncrementalUnit):

    @staticmethod
    def type():
        return "JointVelocityIU"

    def __init__(self, creator=None, iuid=0, previous_iu=None, grounded_in=None, payload: dict[str: list]=None, **kwargs):
        super().__init__(creator=creator, iuid=iuid, previous_iu=previous_iu, grounded_in=grounded_in, payload=payload)
        if payload is not None:
            self.payload = payload
        else:
            self.payload = {}

    def set_single_velocity(self, joint_path, vel):
        self.payload[joint_path] = vel

    def set_multi_velocity(self, list_of_joint_paths, list_of_velocities):
        for path, vel in list(zip(list_of_joint_paths, list_of_velocities)):
            self.payload[path] = vel

    def __str__(self):
        return f"(JointVelocityIU: {self.payload.items()})"


class JointPositionIU(retico_core.abstract.IncrementalUnit):

    @staticmethod
    def type():
        return "JointPositionIU"

    def __init__(self, creator=None, iuid=0, previous_iu=None, grounded_in=None, payload: dict[str: list]=None, **kwargs):
        super().__init__(creator=creator, iuid=iuid, previous_iu=previous_iu, grounded_in=grounded_in, payload=payload)
        if payload is not None:
            self.payload = payload
        else:
            self.payload = {}

    def set_single_position(self, joint_path, pos):
        self.payload[joint_path] = pos

    def set_multi_position(self, list_of_joint_paths, list_of_positions):
        for path, pos in list(zip(list_of_joint_paths, list_of_positions)):
            self.payload[path] = pos

    def __str__(self):
        return f"(JointPositionIU: {self.payload.items()})"


class CoppeliaModule(retico_core.AbstractConsumingModule):

    @staticmethod
    def name():
        return "CoppeliaModule"

    @staticmethod
    def description():
        return "A Controller Module for CoppeliaSim"

    @staticmethod
    def input_ius():
        return [JointPositionIU, JointVelocityIU, JointForceIU]

    @staticmethod
    def output_iu():
        return None

    def __init__(self, scene, start_scene=False, **kwargs):
        super().__init__(**kwargs)

        self.start_scene = start_scene
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        self.sim.loadScene(scene)

        if self.start_scene:
            print("Starting simulation...")
            self.sim.startSimulation()

    def process_update(self, update_message):
        for iu, um in update_message:
            if um == retico_core.abstract.UpdateType.ADD:
                self.process_iu(iu)

    def process_iu(self, iu):
        if type(iu) is JointForceIU:
            for path, force in iu.payload.items():
                handle = self.sim.getObject(path)
                self.sim.setJointTargetForce(handle, force)
        elif type(iu) is JointVelocityIU:
            for path, vel in iu.payload.items():
                handle = self.sim.getObject(path)
                self.sim.setJointTargetVelocity(handle, vel)
        elif type(iu) is JointPositionIU:
            for path, pos in iu.payload.items():
                handle = self.sim.getObject(path)
                self.sim.setJointTargetPosition(handle, pos)

    def shutdown(self):
        if self.start_scene:
            self.sim.stopSimulation()