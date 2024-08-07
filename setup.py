import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'giskardpy_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simon Stelter',
    maintainer_email='stelter@uni-bremen.de',
    description='TODO: Package description',
    license='LGPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generic_giskard = scripts.generic_giskard:main',
            'generic_giskard_standalone = scripts.generic_giskard_standalone:main',
            'interactive_marker = scripts.tools.interactive_marker:main'
        ],
    },
)
