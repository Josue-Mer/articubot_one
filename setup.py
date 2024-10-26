from setuptools import setup
from glob import glob
import os

package_name = 'articubot_one'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'config'), glob('config/*')),
    (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    (os.path.join('share', package_name, 'src'), glob('src/articubot_one/*.py')),
    (os.path.join('share', package_name, 'description'), glob('description/*')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Josh Newans',
    maintainer_email='my_email@email.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_button_action = articubot_one.joy_button_action:main',
        ],
    },
)