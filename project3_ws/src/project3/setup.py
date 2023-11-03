from setuptools import find_packages, setup

package_name = 'project3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scikit-learn'],
    python_requires='>=3.6',
    zip_safe=True,
    maintainer='samuel',
    maintainer_email='samuel.fafel@gmail.com',
    description='CSCE 452 Project 3 - LIDAR Sensor Data Interpretation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = project3.detector:main',
            'counter = project3.counter:main',
    ],
},

)
