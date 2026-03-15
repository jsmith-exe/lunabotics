from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'basestation'

setup(
    name='basestation',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    data_files=[
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    entry_points={
        'console_scripts': [
            'nav_pub = basestation.nodes.controls_publisher:main',
            'nav_sub = basestation.nodes.controls_subscriber_test:main',
        ],
    },
)