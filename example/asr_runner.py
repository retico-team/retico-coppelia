import sys, os

prefix = '<path-to-Retico-repositories>'
sys.path.append(prefix+'retico-core')
sys.path.append(prefix+'retico-whisperasr')
sys.path.append(prefix+'retico-zmq')

from retico_core.audio import MicrophoneModule
from retico_core.debug import DebugModule
from retico_zmq.zmq import WriterSingleton, ZeroMQWriter
from retico_whisperasr import WhisperASRModule

ip = '<local-ip>'
WriterSingleton(ip=ip, port='12345')
zmq_writer = ZeroMQWriter(topic='asr')

mic = MicrophoneModule(rate=16000)
asr = WhisperASRModule()
debug = DebugModule(print_payload_only=True)

mic.subscribe(asr)
asr.subscribe(zmq_writer)
asr.subscribe(debug)

mic.run()
asr.run()
zmq_writer.run()
debug.run()

input()

mic.stop()
asr.stop()
zmq_writer.stop()
debug.stop()
