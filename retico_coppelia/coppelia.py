from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import retico_core


class CoppeliaJointIU(retico_core.abstract.IncrementalUnit):

    @staticmethod
    def type():
        return "CoppeliaJointIU"

    def __init__(self, creator=None, iuid=0, previous_iu=None,
                 grounded_in=None, payload=None, **kwargs):
        super().__init__(creator=creator, iuid=iuid, previous_iu=previous_iu,
                         grounded_in=grounded_in, payload=payload)
        self.payload = payload
        self._vision_loop_active = False

    def __str__(self):
        return "(CoppeliaJointIU: " + self.payload + ")"


class CoppeliaSimController(retico_core.AbstractConsumingModule):

    @staticmethod
    def name():
        return "CoppeliaSimController"

    @staticmethod
    def description():
        return "A Controller Module for CoppeliaSim"

    @staticmethod
    def input_ius():
        return [CoppeliaJointIU]

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
        for joint in iu.payload:
            handle = self.sim.getObject(joint)

            if 'vel' in iu.payload[joint]:
                vel = iu.payload[joint]['vel']
                print(handle, vel)
                self.sim.setJointTargetVelocity(handle, vel)

            if 'pos' in iu.payload[joint]:
                pos = iu.payload[joint]['pos']
                print(handle, pos)
                self.sim.setJointTargetPosition(handle, pos)

    def _set_joint_pos(self, handle, pos):
        self.sim.setJointTargetPosition(handle, pos)

    def _set_joint_vel(self, handle, vel):
        self.sim.setJointTargetVelocity(handle, vel)

    def shutdown(self):
        if self.start_scene:
            self.sim.stopSimulation()