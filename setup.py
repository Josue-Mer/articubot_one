from setuptools import setup
from glob import glob
import os

package_name = 'articubot_one'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'articubot_one.laser_scan_filter'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_scan_filter = articubot_one.laser_scan_filter:main',
        ],
    },
)
