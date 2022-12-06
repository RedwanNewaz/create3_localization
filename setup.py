from setuptools import setup
import os
from glob import glob
package_name = 'create3_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robo',
    maintainer_email='aredwann@uno.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_viz = create3_localization.state_visulizer:main',
            'robot_state = create3_localization.state_fusion:main' ,
            'sync_state = create3_localization.state_odom:main' 
        ],
    },
)
