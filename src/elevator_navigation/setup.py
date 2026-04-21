from setuptools import find_packages, setup

package_name = 'elevator_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neo',
    maintainer_email='doanhbq@pheniakaa-x.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'free_space_goal = elevator_navigation.free_space_goal:main',
            'elevator_visualizer = elevator_navigation.elevator_visualizer:main',
            'simple_gui = elevator_navigation.simple_gui:main',
        ],
    },
)
