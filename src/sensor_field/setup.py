import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sensor_field'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neo',
    maintainer_email='neo.yuichiro@megachips.co.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'csv_sensor_field = sensor_field.csv_sensor_field:main',
            'rf_field = sensor_field.rf_field:main',
            'sensor_field_pose2d = sensor_field.sensor_field_pose2d:main',
            'sample_2d = sensor_field.sample_2d:main',
        ],
    },
)
