from setuptools import setup
from glob import glob
import os

package_name = 'fp_glassguard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    entry_points={
        'console_scripts': [
            'glassguard_node = fp_glassguard.glassguard_node:main',
        ],
    },
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='sevnce',
    maintainer_email='fusion@example.com',
    description='GlassGuard four-stage glass perception pipeline',
    license='MIT',
)
