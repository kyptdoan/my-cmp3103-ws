from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cube_pushing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ky.phuongthu.doan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine = cube_pushing.task1_2.state_machine:main',
            'go_to_pose = cube_pushing.task1_2.go_to_pose:main',
            'push_a_cube = cube_pushing.task1_2.push_a_cube:main',
            'estimate_pose = cube_pushing.task1_2.estimate_pose:main',
            'nav_and_return = cube_pushing.real_robot_task.nav_and_return:main',
        ],
    },
)
