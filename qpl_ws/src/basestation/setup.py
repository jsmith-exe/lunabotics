from setuptools import setup, find_packages

package_name = 'basestation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'nav_pub = basestation.nodes.controls_publisher:main',
            'nav_sub = basestation.nodes.controls_subscriber_test:main',
        ],
    },
)