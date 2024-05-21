from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot4_johnny_lab'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ivan Berman',
    maintainer_email='fingerling42@proton.me',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'johnny_lab_navigator = turtlebot4_johnny_lab.johnny_lab_navigator:main',
            'johnny_lab_robonomics = turtlebot4_johnny_lab.johnny_lab_robonomics:main',
        ],
    },
)
