from setuptools import find_packages, setup
import os, glob

package_name = 'rescue_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob.glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, "rviz"), glob.glob('rviz/*.rviz')),
        (os.path.join('share', package_name, "urdf"), glob.glob('urdf/*')),
        (os.path.join('share', package_name, "config"), glob.glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhj',
    maintainer_email='happyijun@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_move = rescue_robot.simple_move:main',
            'spawn_robot = rescue_robot.spawn_robot:main',
            'simple_move_0pos = rescue_robot.simple_move_0pos:main',
        ],
    },
)
