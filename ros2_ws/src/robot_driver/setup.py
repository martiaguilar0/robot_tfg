import os
from glob import glob
from setuptools import setup

package_name = 'robot_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/robot_driver/launch', ['launch/robot_full.launch.py']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu@email.com',
    description='Driver para robot con STM32',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32_bridge_node=robot_driver.stm32_bridge:main',
            'inv_kinematics_node=robot_driver.inv_kinematics:main',
            'odometry_node=robot_driver.odometry:main',
            'vision_localization_node=robot_driver.vision_localization:main',
            'ekf_node=robot_driver.ekf:main',
            'navigation_node=robot_driver.navigation:main',
        ],
    },
)