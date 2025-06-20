from setuptools import find_packages, setup

package_name = 'my_slam_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam_with_kinect_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roomba',
    maintainer_email='roomba@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
