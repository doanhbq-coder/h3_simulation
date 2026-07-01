import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'map_drift_monitor'

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
    description='Monitor map localization drift and request pose reset when threshold exceeded',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_drift_monitor_node = map_drift_monitor.map_drift_monitor_node:main',
            'map_drift_likelihood_node = map_drift_monitor.map_drift_likelihood_node:main',
            'map_drift_raycast_node = map_drift_monitor.map_drift_raycast_node:main',
            'map_drift_corrector_node = map_drift_monitor.map_drift_corrector_node:main',
        ],
    },
)
