from setuptools import setup

package_name = 'bridge_service'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/mapping.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VienLL',
    maintainer_email='vienll@example.com',
    description='ROS2 bridge service for Yamcs',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'bridge_node = bridge_service.bridge_node:main',
        ],
    },
)
