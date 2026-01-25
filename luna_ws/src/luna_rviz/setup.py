from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'luna_rviz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", "luna_sim", "description"), glob("description/*")),
    (os.path.join("share", "luna_rviz", "launch"), glob("launch/*")),
    (os.path.join("share", "luna_sim", "rviz"), glob("rviz/*")), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='js',
    maintainer_email='jamiesmith.exe@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
