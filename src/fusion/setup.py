from setuptools import setup
import os
from glob import glob

package_name = 'fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install Launch Files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='rosuser@todo.todo',
    description='Sensor fusion package for UAV stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],  
    },
)
