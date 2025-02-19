import threading
import time

import numpy as np
from collections import deque
import retico_core
from retico_core.text import SpeechRecognitionIU
from retico_coppelia.coppelia_cozmo import CoppeliaCozmoIU
from retico_coppelia.coppelia_cozmo_util import Radians, Rads, Millimeters, MMPS


class ASR2CozmoModule(retico_core.AbstractModule):

    @staticmethod
    def name():
        return "ASR2CozmoModule"

    @staticmethod
    def description():
        return "An example module converting SpeechRecognitionIUs to CoppeliaCozmoIUs."

    @staticmethod
    def input_ius():
        return [SpeechRecognitionIU]

    @staticmethod
    def output_iu():
        return CoppeliaCozmoIU

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.queue = deque(maxlen=10)
        self._process = False

    def process_update(self, update_message):
        for iu, ut in update_message:
            if ut == retico_core.abstract.UpdateType.ADD:
                self.queue.append(iu)
            elif ut == retico_core.abstract.UpdateType.REVOKE:
                if iu in self.queue:
                    self.queue.remove(iu)

    def _process_words(self):
        while self._process:
            if len(self.queue) == 0: continue

            sentence = ' '.join([iu.payload for iu in self.queue]).lower()
            output_iu = self.create_iu()

            if 'turn left' in sentence:
                output_iu.payload = {'turn': [Radians(np.pi / 2), Rads(np.pi), True]}
            elif 'turn right' in sentence:
                output_iu.payload = {'turn': [Radians(-np.pi / 2), Rads(np.pi), True]}
            elif 'drive forward' in sentence:
                output_iu.payload = {'drive': [Millimeters(400), MMPS(200), True]}
            elif 'drive backward' in sentence:
                output_iu.payload = {'drive': [Millimeters(-400), MMPS(200), True]}
            elif 'lift up' in sentence:
                output_iu.payload = {'lift': [1, Rads(1), True]}
            elif 'lift down' in sentence:
                output_iu.payload = {'lift': [0, Rads(1), True]}
            elif 'look up' in sentence:
                output_iu.payload = {'look': [1, Rads(1), True]}
            elif 'look down' in sentence:
                output_iu.payload = {'look': [0, Rads(1), True]}
            else:
                time.sleep(0.1)
                continue

            self.queue.clear()
            update_message = retico_core.UpdateMessage.from_iu(output_iu, retico_core.UpdateType.ADD)
            self.append(update_message)

    def setup(self):
        self._process = True
        threading.Thread(target=self._process_words, daemon=True).start()

    def shutdown(self):
        self._process = False
