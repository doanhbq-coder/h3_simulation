from setuptools import setup
import os
from glob import glob

package_name = 'pitag_setpose'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='doanhbq',
    maintainer_email='roboticsvn.ai@gmail.com',
    description='Set robot initial pose via ceiling PiTag detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'setpose_node = pitag_setpose.setpose_node:main',
        ],
    },
)
