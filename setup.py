from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'imars_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include Custom Folders
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matthias',
    maintainer_email='mstarck@outlook.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'JoyToCmdVelNode = imars_bringup.joy_to_cmd_vel_function:main',
            'SerialInterfaceNode = imars_bringup.serial_interface_function:main',
            'I2CInterfaceNode = imars_bringup.i2c_interface_function:main',
            'TwistToAckermannNode = imars_bringup.twist_to_ackermann_function:main',
        ],
    },
)
