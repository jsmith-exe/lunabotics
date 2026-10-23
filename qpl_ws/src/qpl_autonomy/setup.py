import os
from glob import glob

from setuptools import setup

package_name = 'qpl_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.nav'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch/nav', glob('launch/nav/*.launch.py')),
        ('share/' + package_name + '/config/nav', glob('config/nav/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kv',
    maintainer_email='kv@email.com',
    description='Mission-level autonomy for QPL rover',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomy_node = qpl_autonomy.autonomy_node:main',
            'localised_autonomy_node = qpl_autonomy.localised_autonomy_node:main',
            'timed_autonomy_node = qpl_autonomy.timed_autonomy_node:main',
            'nav_autonomy_node = qpl_autonomy.nav.nav_autonomy_node:main',
        ],
    },
)