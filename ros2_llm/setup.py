from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_llm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scferro',
    maintainer_email='stephencferro@gmail.com',
    description='Package for interacting with local LLM and VLM through ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ros2_llm_node = ros2_llm.ros2_llm_node:main",
        ],
    },
    package_data={
        'ros2_llm': ['ros2_llm_interfaces'],
    },
)