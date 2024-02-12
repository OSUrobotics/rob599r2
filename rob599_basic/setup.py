import os
from glob import glob

from setuptools import setup

package_name = 'rob599_basic'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bill Smart',
    maintainer_email='smartw@oregonstate.edu',
    description='ROB599 ROS 2 Examples',
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = rob599_basic.publisher:main',
            'subscriber = rob599_basic.subscriber:main',
            'service = rob599_basic.service:main',
            'client = rob599_basic.service_client:main',
            'action_server = rob599_basic.action_server:main',
            'action_client = rob599_basic.action_client:main',
            'doubler = rob599_basic.multiplier:doubler',
            'noiser = rob599_basic.multiplier:noiser',
        ],
    },
)
