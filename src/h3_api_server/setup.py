from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'h3_api_server'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn[standard]',
        'pydantic',
    ],
    zip_safe=True,
    maintainer='neo',
    maintainer_email='doanhbq@pheniakaa-x.com',
    description='HTTP/WebSocket API server for H3 robot control via ROS2/Nav2',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'h3_api_server = h3_api_server.api_server_node:main',
        ],
    },
)
