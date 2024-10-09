import asyncio

from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, DurabilityPolicy, QoSProfile
from rclpy.node import Node
from picamera2 import Picamera2
from ffmpeg_image_transport_msgs.msg import FFMPEGPacket
from termcolor import colored as c

from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
import libcamera

from .packet_output import PacketsOutput

class Camera():
    def __init__(self, cam_info:dict, node:Node, picam2:Picamera2, cofig:dict, log_message_every_sec:float=5.0):
        
        node.get_logger().info(str(cam_info))
        node.get_logger().info(c(f'Setting up cam {cam_info["Model"]} at location {cam_info["Location"]}; id={cam_info["Id"]}, config={str(cofig)}', 'green'))
        
        self.cam_info = cam_info
        self.node = node
        self.picam2 = picam2

        self.hflip = cofig['hflip']
        self.vflip = cofig['vflip']
        self.bitrate = cofig['bitrate']
        self.framerate = cofig['framerate']
        self.log_message_every_sec = log_message_every_sec

        self.config = cofig
        
    async def start(self, topic_prefix):
        self.topic = topic_prefix + str(self.cam_info["Location"]) + '/' + self.cam_info["Model"]
        
        qos = QoSProfile(history=self.config['history'], \
                        depth=self.config['depth'], \
                        reliability=self.config['reliability'], \
                        durability=self.config['durability'] \
                        )
        self.pub = self.node.create_publisher(FFMPEGPacket, self.topic, qos)
        if self.pub == None:
            self.node.get_logger().error(f'Failed to create publisher for cam {self.cam_info["Model"]} topic {self.topic}')
            
        self.node.get_logger().info(c(f'Created publisher for cam {self.cam_info["Model"]} at {self.topic}, qos={qos}', 'green'))
    
        transform = libcamera.Transform(hflip=1 if self.hflip else 0, vflip=1 if self.vflip else 0)
        video_config = self.picam2.create_video_configuration(queue=False, transform=transform)
        self.picam2.configure(video_config)
        self.encoder = H264Encoder(bitrate=self.bitrate, framerate=self.framerate)
        self.output = PacketsOutput(enc=self.encoder, pub=self.pub, cam_info=self.cam_info, node=self.node, log_message_every_sec=self.log_message_every_sec)
        # self.peers[id_peer].track.set_output(self.output)

        await asyncio.sleep(2.0) #camera setup time (here?)

        self.node.get_logger().info(c(f'Picam2 {self.cam_info["Model"]} streaming...', 'magenta'))

        # picam2.start_recording()
        self.picam2.start_encoder(encoder=self.encoder, output=self.output)
        self.picam2.start()
        
    async def stop(self):
        self.node.get_logger().info(f'Destroying local publisher for {self.topic}')

        self.pub.destroy()
        self.pub = None
        self.topic = None
        