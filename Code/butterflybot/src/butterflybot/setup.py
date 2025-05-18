from setuptools import setup
from glob import glob
import os

package_name = 'butterflybot'

setup(
    name=package_name,
    version='0.0.0',#origin:1.0.0
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'world'), glob('world/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'cartographer/config'), glob('cartographer/config/*.lua')),
        (os.path.join('share', package_name, 'cartographer/launch'), glob('cartographer/launch/*.py')),
    ],
    install_requires=[
    'setuptools',
    'rclpy',
    'python_qt_binding',
    ],
    zip_safe=True,
    maintainer='butterflybot',
    maintainer_email='your-email@example.com',
    description='URDF description package for butterflybot. Includes launch files, RViz and Gazebo configurations.',
    license='Apache License 2.0', #'BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'butterfly_node = butterflybot.butterfly_node:main',
            'transform_broadcaster = butterflybot.transform_broadcaster:main',
            'object_detection_node = butterflybot.object_detection_node:main',
            #'cartographer_node = butterflybot.cartographer_node:main',
            'joint_controller = butterflybot.joint_controller:main',  # Entry point for the joint_controller script
        ],
        'rviz2.panel_plugins': [
        'MapSavePanel = butterflybot.map_save_panel:MapSavePanel',
        ],
    },
)
