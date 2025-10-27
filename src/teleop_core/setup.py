from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'teleop_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rover2-i',
    maintainer_email='rover2-i@todo.todo',
    description='TODO: Package description',
    license='ECL-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy2cmd = teleop_core.run_joy_cmd:main',
            'cmd_demux = teleop_core.demux:main',
            'joywithgui = teleop_core.run_joy_with_gui:main',
            'joywithgui3 = teleop_core.run_joy_with_gui_3:main',
        ],
    },
)
