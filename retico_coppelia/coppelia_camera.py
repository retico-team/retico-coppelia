import threading
import cv2
import numpy as np
from PIL import Image
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import retico_core
from retico_vision.vision import ImageIU


class CoppeliaSimCamera(retico_core.AbstractProducingModule):

    @staticmethod
    def name():
        return "CoppeliaSimCamera"

    @staticmethod
    def description():
        return "A camera module for CoppeliaSim that produces virtual images"

    @staticmethod
    def input_ius():
        return None

    @staticmethod
    def output_iu():
        return ImageIU

    def __init__(self, scene, start_scene=False, sensor_path=None, visualizer=False, **kwargs):
        super().__init__(**kwargs)

        if sensor_path is None:
            raise Exception("No CoppeliaSim sensor path specified.")

        self.start_scene = start_scene
        self.sensor_path = sensor_path
        self._vision_loop_active = False
        self.visualizer = visualizer

        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')

        if start_scene:
            self.sim.loadScene(scene)
            self.sim.startSimulation()

    def process_update(self, um):
        return None

    def _vision_loop(self):
        handle = self.sim.getObject(self.sensor_path)
        while self._vision_loop_active:
            img_buffer, res = self.sim.getVisionSensorImg(handle)
            img = np.frombuffer(img_buffer, dtype=np.uint8).reshape(res[1], res[0], 3)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = cv2.flip(img, 0)

            if self.visualizer:
                cv2.imshow(self.sensor_path, img)

                k = cv2.waitKey(1) & 0xFF
                if k == 27:
                    cv2.destroyAllWindows()
                    break

            frame = Image.fromarray(img)
            output_iu = self.create_iu()
            output_iu.set_image(frame, 1, -1)

            update_message = retico_core.UpdateMessage.from_iu(output_iu, retico_core.UpdateType.ADD)
            self.append(update_message)

    def setup(self):
        self._vision_loop_active = True
        t = threading.Thread(target=self._vision_loop)
        t.start()

    def shutdown(self):
        self._vision_loop_active = False

        cv2.destroyAllWindows()

        if self.start_scene:
            self.sim.stopSimulation()