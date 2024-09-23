from setuptools import find_packages, setup

package_name = 'bridge_pubsub'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='haha123@gmail.com',
    description='Simple Listener to ROS 1 Values',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = bridge_pubsub.publisher_function:main',
            'listener = bridge_pubsub.subscriber_function:main'
        ],
    },
)
