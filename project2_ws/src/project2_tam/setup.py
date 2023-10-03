from setuptools import find_packages, setup

package_name = 'project2_tam'

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
        	'background = project2_tam.background:main',
        	'project2 = project2_tam.project2:main',
        	'tam_logo = project2_tam.tam_logo:main',
        ],
    },
)
