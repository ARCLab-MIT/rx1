from setuptools import setup, find_packages

setup(
    name='rx1_motor',
    version='1.2.5',
    packages=find_packages(),
    package_dir={'': 'motor_control'},
    install_requires=[
        'pyserial',
        'pyyaml'
    ],
)