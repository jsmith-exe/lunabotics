from setuptools import setup, find_packages

setup(
    name='basestation',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'nav_pub = basestation.nodes.controls_publisher:main',
            'nav_sub = basestation.nodes.controls_subscriber_test:main',
        ],
    },
)