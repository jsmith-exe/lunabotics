from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'basestation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jamie Smith',
    maintainer_email='your@email.com',
    description='Basestation package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = basestation.main:main',
            'log_recorder = basestation.nodes.log_recorder:main',
            'zone_overlay = basestation.nodes.zone_overlay_node:main',
        ],
    },
)