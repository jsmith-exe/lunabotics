from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qpl_rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ("share/ament_index/resource_index/packages", ["resource/qpl_rover"]),
    ("share/qpl_rover", ["package.xml"]),
    (os.path.join("share",  package_name, "description"), glob("description/*")),
    (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    (os.path.join("share", package_name, "launch"), glob("launch/components/*.py")),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    (os.path.join('share', package_name, 'worlds', 'apriltag_model'), glob('worlds/apriltag_model/model.config')),
    (os.path.join('share', package_name, 'worlds', 'apriltag_model', 'materials', 'scripts'), glob('worlds/apriltag_model/materials/scripts/*')),
    (os.path.join('share', package_name, 'worlds', 'apriltag_model', 'materials', 'textures'), glob('worlds/apriltag_model/materials/textures/*')),
    (os.path.join('share', package_name, 'config'), glob('config/*')),
    (os.path.join("share", package_name, "rviz"), glob("rviz/*")), 
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
            'apriltag_pose_2d = qpl_rover.nodes.apriltag_pose_2d:main',
            'apriltag_map_odom = nodes.apriltag_map_odom:main',
            'apriltag_map_odom_3d = nodes.apriltag_map_odom_3d:main',
            'apriltag_tag_base = nodes.apriltag_tag_base:main',
        ],
    },
)