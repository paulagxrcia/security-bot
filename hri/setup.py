from setuptools import find_packages, setup
from glob import glob

package_name = 'hri'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/data', glob('data/*.*')),
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
            'lower_body_detection = hri.lower_body_detection:main',
            'full_body_detection = hri.full_body_detection:main',
            'human_tracking = hri.human_tracking:main'
        ],
    },
)
