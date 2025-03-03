from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
d = generate_distutils_setup(
    name='rx1_motor',
    version='1.2.5',
    packages=find_packages(where='motor_control'),
    package_dir={'': 'motor_control'},
    install_requires=[
        'pyserial',
        'pyyaml'
    ],
)

setup(**d)