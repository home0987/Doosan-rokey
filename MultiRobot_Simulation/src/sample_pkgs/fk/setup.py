from setuptools import find_packages, setup

package_name = 'fk'

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
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'arm = fk.arm:main',
        'arm_gripper = fk.arm_gripper:main',
        'gripper = fk.gripper:main',
        'moveit = fk.moveit:main',
        ],
    },
)
