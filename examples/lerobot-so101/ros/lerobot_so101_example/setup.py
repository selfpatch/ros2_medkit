# Copyright 2026 ahmedsleem109
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from glob import glob

from setuptools import setup

package_name = 'lerobot_so101_example'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmedsleem109',
    maintainer_email='ahmedsleem8843@gmail.com',
    description='LeRobot ACT policy on a simulated SO-101 arm, diagnosed by ros2_medkit',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'so101_sim_node = lerobot_so101_example.so101_sim_node:main',
            'lerobot_policy_node = lerobot_so101_example.lerobot_policy_node:main',
            'camera_monitor = lerobot_so101_example.camera_monitor:main',
            'episode_runner = lerobot_so101_example.episode_runner:main',
        ],
    },
)
