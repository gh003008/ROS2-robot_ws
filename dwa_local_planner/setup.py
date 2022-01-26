from setuptools import setup
from setuptools import find_packages
import glob
import os

package_name = 'dwa_local_planner'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        (share_dir + '/param', glob.glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leegunhee1',
    maintainer_email='gh003008@naver.com',
    description='turtlesim using dwa',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
 	'dwa_planner = dwa_local_planner.dwa:main'
        ],
    },
)
