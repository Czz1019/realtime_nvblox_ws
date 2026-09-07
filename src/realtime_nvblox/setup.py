from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'realtime_nvblox'

setup(
    name=package_name,
    version='1.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'DEPLOY.md', 'LICENSE']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    package_data={package_name: ['default.yaml']},
    install_requires=['setuptools', 'PyYAML', 'numpy', 'scipy'],
    zip_safe=False,
    maintainer='czz',
    maintainer_email='czz@example.com',
    description='Python-first RealSense + cuVSLAM + nvblox_torch realtime 3D reconstruction.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'standalone_mapper = realtime_nvblox.cli:main',
            'ros_mapper = realtime_nvblox.ros_node:main',
            'check_environment = realtime_nvblox.check_env:main',
        ],
    },
)
