from setuptools import setup

package_name = 'point_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/point_nav.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Point-to-point navigation via Joy commands using visual odometry',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_nav_node = point_nav.point_nav_node:main',
        ],
    },
)

