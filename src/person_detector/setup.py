from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'person_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neo',
    maintainer_email='doanhbq@pheniakaa-x.com',
    description='Person detection from 2D LiDAR for receptionist robot greeting',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_detector_node = person_detector.person_detector_node:main',
            'greeting_node = person_detector.greeting_node:main',
            'scan_debugger = person_detector.scan_debugger:main',
        ],
    },
)
