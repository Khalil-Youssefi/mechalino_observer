from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mechalino_observer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khalil',
    maintainer_email='kh4lil@outlook.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'observer = mechalino_observer.observer:main',
            'cam2topic = mechalino_observer.cam2topic:main',
            'undistorted_img_pub = mechalino_observer.undistorted_img_pub:main',
            'pose_estimator = mechalino_observer.pose_estimator:main'
        ],
    },
)
