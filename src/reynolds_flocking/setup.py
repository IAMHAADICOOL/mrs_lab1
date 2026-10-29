import os
from glob import glob

from setuptools import setup

package_name = 'reynolds_flocking'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haadi',
    maintainer_email='tasneemkauseramu@gmail.com',
    description='Reynolds flocking rules with obstacle avoidance for the Stage simulator',
    license='MIT',
    entry_points={
        'console_scripts': [
            'flocking_node = reynolds_flocking.flocking_node:main',
            'plot_flocking = reynolds_flocking.plot_flocking:main',
        ],
    },
)
