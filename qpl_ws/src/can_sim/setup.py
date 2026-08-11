from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'can_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Caleb Hair',
    maintainer_email='calebthair@outlook.com',
    description='CAN Simulation',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'can_sim = can_sim.main:main',
        ],
    },
)
