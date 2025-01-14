from setuptools import setup

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your.email@example.com',
    description='Navigation package for delivery robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_navigation = navigation.node_navigation:main',
            'node_obstacle_avoidance = navigation.node_obstacle_avoidance:main',
        ],
    },
)
