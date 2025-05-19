from setuptools import find_packages, setup
import os, glob

package_name = 'rescue_turtlebot3_bringup'

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
        (os.path.join('share', package_name, "map"), glob.glob('map/*')),
        (os.path.join('share', package_name, "worlds"), glob.glob('worlds/*')),
        (os.path.join('share', package_name, "models"), glob.glob('models/*')),
        (os.path.join('share', package_name, "param"), glob.glob('param/*')),
        #(os.path.join('share', package_name, "waypoints"), glob('waypoints/*.yaml')),
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
            'spawn_ran = rescue_turtlebot3_bringup.spawn_ran:main',
        ],
    },
)
