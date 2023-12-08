from setuptools import find_packages, setup

package_name = 'project4'

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
    maintainer='samuel',
    maintainer_email='samuel.fafel@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator = project4.simulator:main',
            'velocity_translator = project4.velocity_translator:main',
            'navigation_controller = project4.navigation_controller:main',
            'robot_state_publisher = robot_state_publisher:main'
        ],
    },
)
