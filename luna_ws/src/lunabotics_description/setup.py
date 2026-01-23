from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'lunabotics_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ("share/ament_index/resource_index/packages", ["resource/lunabotics_description"]),
    ("share/lunabotics_description", ["package.xml"]),
    (os.path.join("share", "lunabotics_description", "urdf"), glob("urdf/*")),
    (os.path.join("share", "lunabotics_description", "launch"), glob("launch/*")),
    (os.path.join("share", "lunabotics_description", "rviz"), glob("rviz/*")), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='js',
    maintainer_email='js@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [],
    },
)
