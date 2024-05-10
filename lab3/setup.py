from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lab3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name, "config"), glob('config/*')),
        (os.path.join('share', package_name, "urdf"), glob('urdf/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mxn',
    maintainer_email='mxn@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forward_kin = lab3.forward_kin:main',
            'joint_state_publisher = lab3.joint_state_publisher:main',
            'joint_states_conv = lab3.joint_states_conv:main'
        ],
    },
)
