from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drawbot_moveit_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        ('share/' + package_name + '/srdf', glob('srdf/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/environment', glob('environment/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrewda',
    maintainer_email='dassonville.andrew@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    },
)
