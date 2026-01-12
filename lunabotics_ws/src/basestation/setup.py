from setuptools import setup, find_packages

setup(
    name='basestation',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'test = basestation.test:main',
            # Add other node entry points here
        ],
    },
)