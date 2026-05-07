import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'auto_runner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neo',
    maintainer_email='yneo918@gmail.com',
    description='Headless batch experiment runner for adaptive navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'headless_an_controller = auto_runner.headless_an_controller:main',
            'batch_orchestrator = auto_runner.batch_orchestrator:main',
        ],
    },
)
