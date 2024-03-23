from setuptools import find_packages, setup

package_name = 'picam_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mirek Burkon',
    maintainer_email='mirek@phntm.io',
    description='Picamera h.264 frames to ROS2 messages',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'picam_ros2 = picam_ros2.picam_ros2:main'
        ],
    },
)
