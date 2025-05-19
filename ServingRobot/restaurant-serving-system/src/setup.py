from setuptools import find_packages, setup

package_name = 'test_node'

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
    maintainer='yujin',
    maintainer_email='yujin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'order_node = test_node.order:main',
            'subscribe_node = test_node.subscribe:main',
            'table_1_node = test_node.table_1:main',
            'table_2_node = test_node.table_2:main',
            'table_3_node = test_node.table_3:main',
            'table_4_node = test_node.table_4:main',
            'table_5_node = test_node.table_5:main',
            'table_6_node = test_node.table_6:main',
            'table_7_node = test_node.table_7:main',
            'table_8_node = test_node.table_8:main',
            'table_9_node = test_node.table_9:main',
            'gui=turtlebot3_gui.gui_test:main',
        ],
    },
)
