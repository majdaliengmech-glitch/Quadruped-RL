from glob import glob

from setuptools import find_packages, setup

package_name = 'policy_node'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='majd-ali',
    maintainer_email='majd30706@gmail.com',
    description='Walking policy, high-rate joint PD controller and odometry TF node',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'policy_node = policy_node.policy_node:main',
            'pd_controller = policy_node.pd_controller:main',
            'odom_tf = policy_node.odom_tf:main',
        ],
    },
)