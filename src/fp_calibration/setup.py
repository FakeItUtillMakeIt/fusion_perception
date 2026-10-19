from setuptools import setup
from glob import glob

package_name = 'fp_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='sevnce',
    maintainer_email='fusion@example.com',
    description='Calibration tools',
    license='MIT',
)
