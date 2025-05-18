from setuptools import setup
from glob import glob
import os

package_name = 'butterflybot'
package_name_2 = 'object_recognition_pkg'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='butterflybot',
    maintainer_email='your-email@example.com',
    description='URDF description package for butterflybot. Includes launch files, RViz and Gazebo configurations.',
    license='Apache License 2.0', #'BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'butterfly_node = butterflybot.butterfly_node:main',
            'transform_broadcaster = butterflybot.transform_broadcaster:main',  # If you have this
            'object_detection_node = butterflybot.object_detection_node:main'  # Add this line
        ],
    },
)
