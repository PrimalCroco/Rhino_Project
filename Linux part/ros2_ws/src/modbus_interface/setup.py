from setuptools import find_packages, setup

package_name = 'modbus_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pymodbus'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'modbus_node = modbus_interface.modbus_node:main',
        	'modbus_connect_test = modbus_interface.modbus_connect_test:main',
        	'test = modbus_interface.test:main',
        ],
    },
)
