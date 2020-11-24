from setuptools import setup

package_name = 'rob599_basic_r2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'publisher = rob599_basic_r2.publisher:main',
            'subscriber = rob599_basic_r2.subscriber:main',
            'service = rob599_basic_r2.service:main',
            'client = rob599_basic_r2.service_client:main',
            'action_server = rob599_basic_r2.action_server:main',
            'action_client = rob599_basic_r2.action_client:main',
        ],
    },
)
