from glob import glob

from setuptools import find_packages, setup

package_name = 'go_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='majd-ali',
    maintainer_email='majd30706@gmail.com',
    description='Joystick/twist_mux teleop, SLAM mapping, Nav2 config, goal, patrol and obstacle tools for the Go1',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'patrol = go_navigation.patrol:main',
            'reset_robot = go_navigation.reset_robot:main',
            'goto = go_navigation.goto:main',
            'obstacles = go_navigation.obstacles:main',
        ],
    },
)
