#shared camera frame output
from picamera2.outputs import FileOutput
from rclpy.node import Node
from rclpy.publisher import Publisher
from picamera2.encoders import H264Encoder
import av
import fractions
import time
import math

from ffmpeg_image_transport_msgs.msg import FFMPEGPacket

NS_TO_SEC = 1000000000
# VIDEO_PTIME = 1 / 30  # 30fps
SRC_VIDEO_TIME_BASE = fractions.Fraction(1, NS_TO_SEC)

class PacketsOutput(FileOutput):

    # last_frame: = None
    # last_keyframe:av.Packet = None
    # last_timestamp = 0
    # last_keyframe_timestamp = 0
    # last_was_keyframe = False

    def __init__(self, enc:H264Encoder, pub:Publisher, cam_info:dict, node:Node, log_message_every_sec:float=5.0):
        super().__init__()
        # self.last_frameav.Packet = None
        # self.last_keyframe = None
        # self.last_timestamp = 0
        # elf.last_keyframe_timestamp = 0
        # self.last_was_keyframe = False
       
        self.last_frame:int = 0
        self.num_received = 0
        self.cam_info = cam_info
        self.node = node
        self.pub = pub
        self.enc = enc

        self.last_log:float = -1.0
        self.log_message_every_sec:float = log_message_every_sec

        # self.aiortc_encoder:aiortcH264Encoder() = aiortcH264Encoder()

    def outputframe(self, frame_bytes, keyframe=True, timestamp=None):

        self.num_received += 1

        packet = av.Packet(frame_bytes)
        # packet.pts = timestamp # ns
        # packet.time_base = SRC_VIDEO_TIME_BASE

        log_msg = False
        if self.num_received == 1: # first data in
            log_msg = True
            self.node.get_logger().debug(f'👁️  Receiving data from camera {self.cam_info["Id"]}, {len(frame_bytes)} B last frame')

        if self.last_log < 0 or time.time()-self.last_log > self.log_message_every_sec:
            log_msg = True
            self.last_log = time.time() #last logged now

        # payloads, stamp_converted = self.aiortc_encoder.pack(packet)

        # if self.sub.recording and self.sub.pub:
        #     im = Image()
        #     im.header.frame_id = self.sub.id_camera
        #     im.header.stamp.sec = timestamp // 1_000_000_000
        #     im.header.stamp.nanosec = timestamp % 1_000_000_000
        #     im.width = self.sub.encoder.width
        #     im.height = self.sub.encoder.height
        #     im.encoding = 'h.264'
        #     im.data = frame_bytes
        #     self.sub.pub.publish(im)

        if log_msg:
            self.node.get_logger().info(f'👁️  Sending {self.enc.width}x{self.enc.height} frames from {self.cam_info["Id"]} into {self.pub.topic}, last frame was {len(frame_bytes)} B')

        self.last_frame = timestamp
        
        msg = FFMPEGPacket()
        # time_nanosec:int = time.time_ns()
        msg.header.stamp.sec = math.floor(timestamp / 1000000000)
        msg.header.stamp.nanosec = timestamp % 1000000000  # Ensure nanosec is within the valid range
        msg.width = self.enc.width
        msg.height = self.enc.height
        msg.encoding = 'h.264'
        msg.pts = timestamp # ns # 'uint64',
        msg.flags = 1 if keyframe else 0 # 'uint8',
        msg.is_bigendian = False
        msg.data = frame_bytes
        
        try: 
            self.pub.publish(msg)
        except Exception as e:
            pass
        
        # payload_type = 101 # =variable
        #self.last_frame_tasks[id_peer] = self.sub.event_loop.create_task(self.sub.peers[id_peer].send_direct(frame_data=payloads, stamp_converted=stamp_converted, keyframe=keyframe))
            
