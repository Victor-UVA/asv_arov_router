import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'asv_arov_router'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='malori',
    maintainer_email='abo7fg@virginia.edu',
    description='Package for running data collection, video recording and processing, and controls on the ASV/AROV system in the VICTOR lab',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bluerov_connection = asv_arov_router.bluerov_connection:main',
            'maddy_connection = asv_arov_router.maddy_connection:main',
            'data_logger = asv_arov_router.data_logger:main',
            'bluerov_video_recorder = asv_arov_router.bluerov_video_recorder:main'
        ],
    },
)
