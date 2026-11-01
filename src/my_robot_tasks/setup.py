from setuptools import find_packages, setup

package_name = 'my_robot_tasks'

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
    maintainer='wxj',
    maintainer_email='qwxj@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose_monitor = my_robot_tasks.pose_monitor:main',
            'cmd_vel_repeater = my_robot_tasks.cmd_vel_repeater:main',
            'patrol = my_robot_tasks.patrol:main',
        ],
    },
)
