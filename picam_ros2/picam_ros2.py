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

ROOT = os.path.dirname(__file__)

class CameraNode(Node):
    ##
    # node constructor
    ##
    def __init__(self):
        super().__init__(node_name='picam_ros', use_global_arguments=True)
        
        self.declare_parameter('topic_prefix', '/picam_')
        self.topic_prefix = self.get_parameter('topic_prefix').get_parameter_value().string_value

         
        print(c(f'Hi from CameraNode init, topic prefix is {self.topic_prefix}', 'yellow'))
    
    async def shutdown_cleanup(self):
        pass


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
        from picamera2 import Picamera2
        print(c('Picamera2 loaded', 'green'))
    except Exception as e:
        print(c('Failed to import picamera2', 'red'), e)
        pass
    
    try:
        cam_node = CameraNode()
    except Exception as e:
        print(c('Exception in main_async()', 'red'))
        traceback.print_exc(e)
    except (asyncio.CancelledError, KeyboardInterrupt):
        print(c('Shutting down main_async', 'red'))
        
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
    asyncio.run(main_async())


if __name__ == '__main__':
    main()
