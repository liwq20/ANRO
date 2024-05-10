from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lab4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name, "config"), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz/*'))),
        (os.path.join('share', package_name, "urdf"), glob('urdf/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inverse_kin = lab4.inverse_kin:main',
            'move_to_point = lab4.move_to_point:main',
            'forward_kin = lab4.forward_kin:main',
            'joint_state_publisher = lab4.joint_state_publisher:main'
        ],
    },
)
