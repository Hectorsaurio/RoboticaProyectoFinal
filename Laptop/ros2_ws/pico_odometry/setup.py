from setuptools import find_packages, setup

package_name = 'pico_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hector',
    maintainer_email='hector.fernandez.b@ucb.edu.bo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_to_odom_node = pico_odometry.encoder_to_odom_node:main',
        ],
    },
)
