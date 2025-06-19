from setuptools import find_packages, setup

package_name = 'scripts'

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
    maintainer='paula',
    maintainer_email='garciapaula297@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shelf_inspection = scripts.shelf_inspection_node:main', 
            'security_patrol = scripts.security_patrol_node:main',
            'alarm_publisher = scripts.alarm_alert_node:main',
            'move_joint = scripts.move_joint:main',
            'joint_position_reader = scripts.joint_position_reader:main',
            'joint_pid_controller = scripts.joint_pid_controller:main'

        ],
    },
)
