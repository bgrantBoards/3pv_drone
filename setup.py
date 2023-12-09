from setuptools import find_packages, setup
from glob import glob
import os

package_name = '3pv_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # # install node executables so they are accessible by launch scripts
        # ('lib/' + package_name, ['3pv_drone/crazyflie_motion_control.py']),
        # ('lib/' + package_name, ['3pv_drone/aruco_tag_node.py']),
        # ('lib/' + package_name, ['3pv_drone/image_publisher.py']),

        # install launch scripts
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ben',
    maintainer_email='78234854+bgrantBoards@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = 3pv_drone.test_node:main",
            "image_publisher = 3pv_drone.image_publisher:main",
            "crazyflie_motion_control = 3pv_drone.crazyflie_motion_control:main",
            "aruco_tag_node = 3pv_drone.aruco_tag_node:main",
            "webots_sim_node = 3pv_drone.webots_sim_node:main",
            
        ],
    },
)
