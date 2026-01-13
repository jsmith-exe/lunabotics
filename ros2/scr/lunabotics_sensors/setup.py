from setuptools import find_packages, setup

package_name = 'lunabotics_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
  'console_scripts': [
    'temp_pub = lunabotics_sensors.temp_publisher:main',
    'temp_sub = lunabotics_sensors.temp_subscriber.py',
    'camera_pub = lunabotics_sensors.camera_publisher:main',
    'camera_sub = lunabotics_sensors.camera_subscriber.py',
        ],
    },
)
