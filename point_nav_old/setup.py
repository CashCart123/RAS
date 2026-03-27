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
        (
            'share/' + package_name + '/config',
            ['config/point_nav.defaults.yaml', 'config/goal_inputs.defaults.yaml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CashCart',
    maintainer_email='akshart.amatya@outlook.com',
    description='Point-to-point navigation via Joy commands using camera pose input',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_nav_node = point_nav.point_nav_node:main',
            'set_goal = point_nav.set_goal_cli:main',
        ],
    },
)
