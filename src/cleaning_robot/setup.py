from setuptools import find_packages, setup
import os, glob

package_name = 'cleaning_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob.glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, "config"), glob.glob('config/*')),
        (os.path.join('share', package_name, "imgs"), glob.glob('imgs/*')),
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
            'move_goal = cleaning_robot.move_goal:main',
            'next_point_BFS = cleaning_robot.next_point_BFS:main',
            'next_point_grid = cleaning_robot.next_point_grid:main',
            'frontier = cleaning_robot.frontier:main',
            'camera_img = cleaning_robot.camera_img:main',
            'image_matcher = cleaning_robot.image_matcher:main',
            'camera_map_tf = cleaning_robot.camera_map_tf:main',
        ],
    },
)
