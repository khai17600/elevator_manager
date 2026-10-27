from setuptools import find_packages, setup

package_name = 'elevator_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khai',
    maintainer_email='khai@localhost',
    description='Elevator robot control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go_to_elevator = elevator_robot.go_to_elevator:main',
	        'map_switcher = elevator_robot.map_switcher:main',
            'elevator_manager = elevator_robot.elevator_manager:main',
            'elevator_gui = elevator_robot.elevator_gui:main',
            'elevator_entry_test = elevator_robot.elevator_entry_test:main',
        ],
    },
)
