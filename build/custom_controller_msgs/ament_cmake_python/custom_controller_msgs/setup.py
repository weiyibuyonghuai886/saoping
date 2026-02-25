from setuptools import find_packages
from setuptools import setup

setup(
    name='custom_controller_msgs',
    version='1.0.0',
    packages=find_packages(
        include=('custom_controller_msgs', 'custom_controller_msgs.*')),
)
