from setuptools import setup, find_packages

setup(
    name='basestation',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'pub = basestation.nodes.controls_publisher:main',
            'sub_test = basestation.nodes.sub_test.controls_subscriber_test:main',
        ],
    },
)