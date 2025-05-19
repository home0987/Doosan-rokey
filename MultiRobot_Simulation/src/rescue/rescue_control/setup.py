from setuptools import find_packages, setup
import os, glob

package_name = 'rescue_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob.glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, "mapping"), glob.glob('mapping/*')),
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
            'send_waypoint = rescue_control.send_waypoint:main',
            'GUI = rescue_control.GUI:main',
            'camera_openCV = rescue_control.camera_openCV:main',
            'camera_openCV_nav2 = rescue_control.camera_openCV_nav2:main',
        ],
    },
)
