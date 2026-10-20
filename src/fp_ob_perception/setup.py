from setuptools import setup
from glob import glob
import os

package_name = 'fp_ob_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, glob('scripts/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='sevnce',
    maintainer_email='fusion@example.com',
    description='Obstacle perception module (v2 plan)',
    license='MIT',
)
