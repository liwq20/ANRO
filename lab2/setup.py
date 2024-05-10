from setuptools import find_packages, setup
from glob import glob

package_name = 'lab2'

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
    maintainer='mat',
    maintainer_email='mat@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtle_controller = lab2.turtle_controller:main",
            "pick_and_place = lab2.pick_and_place:main"
        ],
    },
)
