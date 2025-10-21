from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'adaptive_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ricky',
    maintainer_email='rschober@scu.edu',
    description='TODO: Package description',
    license='ECL-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'adaptive_nav = adaptive_nav.adaptive_nav:main'
        ],
    },
)
