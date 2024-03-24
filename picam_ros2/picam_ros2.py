import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration, Infinite
from rclpy.serialization import deserialize_message

from ffmpeg_image_transport_msgs.msg import FFMPEGPacket

from termcolor import colored as c
import asyncio
import selectors
import os
import subprocess
import traceback

try:
    from picamera2 import Picamera2
    print(c('Picamera2 loaded', 'green'))
except Exception as e:
    print(c('Failed to import picamera2', 'red'), e)
    exit(1)

from .camera import Camera

ROOT = os.path.dirname(__file__)

class CameraNode(Node):
    ##
    # node constructor
    ##
    def __init__(self):
        super().__init__(node_name='picam_ros', use_global_arguments=True)
        
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        self.declare_parameter('topic_prefix', '/picam_')
        self.topic_prefix = self.get_parameter('topic_prefix').get_parameter_value().string_value
        
        self.declare_parameter('log_message_every_sec', 5.0)
        self.log_message_every_sec = self.get_parameter('log_message_every_sec').get_parameter_value().double_value
        
        self.camera_configs = {}
        for location in range(1, 5): # configs for individual cameras (i = Location, 1-4)
            self.declare_parameter(f'/camera_{location}.hflip', False)
            self.declare_parameter(f'/camera_{location}.vflip', False)
            self.declare_parameter(f'/camera_{location}.bitrate', 5000000)
            self.declare_parameter(f'/camera_{location}.framerate', 30)
            self.camera_configs[str(location)] = {
                'hflip': self.get_parameter(f'/camera_{location}.hflip').get_parameter_value().bool_value,
                'vflip': self.get_parameter(f'/camera_{location}.vflip').get_parameter_value().bool_value,
                'bitrate': self.get_parameter(f'/camera_{location}.bitrate').get_parameter_value().integer_value,
                'framerate': self.get_parameter(f'/camera_{location}.framerate').get_parameter_value().integer_value
            }
        
        self.cams:list = []
        
        print(c(f'Hi from CameraNode init, topic prefix is {self.topic_prefix}', 'yellow'))
        
        self.picam2 = None
        try:
            self.picam2 = Picamera2()
            cam_infos = self.picam2.global_camera_info()
            num = len(cam_infos)
            if not cam_infos or not num:
                self.get_logger().error('Picamera did\'t find any cameras')
                return
            
            self.get_logger().info(c(f'Picamera found {num} {"cameras" if num > 1 else "camera"}', 'cyan'))
            for cam_info in cam_infos:
                cam = Camera(cam_info, self, self.picam2, self.camera_configs[str(cam_info['Location'])], self.log_message_every_sec)
                self.cams.append(cam)
                asyncio.get_event_loop().create_task(cam.start(self.topic_prefix))
                
        except (Exception, AttributeError) as e:
            print(c('Picamera2 init failed', 'red'))
            print(e)
    
    async def run(self):
        self.running = True
        while self.running:
            await asyncio.sleep(0.1)
    
    async def shutdown_cleanup(self):
        for cam in self.cams:
            await cam.stop()


async def main_async():
    print(c('Hi from picam_ros2 main_async', 'green'))
    
    rclpy.init()

    if not os.path.exists('/ros2_ws/phntm_devices_initialized'):
        print(c('First run, initializing udev rules for /dev (bcs Picam)', 'magenta'))
        process = subprocess.Popen([f'{ROOT}/../scripts/reload-devices.sh'])
        process.wait()
        print(c('Udev rules initialized', 'magenta'))
        await asyncio.sleep(1.0) # needs a bit for the udev rules to take effect and picam init sucessfuly

    try:
        cam_node = CameraNode()
        stream_task = cam_node.run()
        await asyncio.wait([ stream_task ], return_when=asyncio.ALL_COMPLETED)
    except Exception as e:
        print(c('Exception in main_async()', 'red'))
        traceback.print_exc(e)
        cam_node.running = False
    except (asyncio.CancelledError, KeyboardInterrupt):
        print(c('Shutting down main_async', 'red'))
        cam_node.running = False
        pass
        
    print('SHUTTING DOWN')
    await cam_node.shutdown_cleanup()
    
    try:
        cam_node.destroy_node()
        rclpy.shutdown()
    except:
        pass
    

class MyPolicy(asyncio.DefaultEventLoopPolicy):
    def new_event_loop(self):
        selector = selectors.SelectSelector()
        return asyncio.SelectorEventLoop(selector)


def main():
    asyncio.set_event_loop_policy(MyPolicy())
    try:
        asyncio.run(main_async())
    except (asyncio.CancelledError, KeyboardInterrupt):
        pass


if __name__ == '__main__':
    main()
