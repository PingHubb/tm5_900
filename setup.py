from setuptools import find_packages, setup

package_name = 'lan_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'arcade'],
    zip_safe=True,
    maintainer='ping2',
    maintainer_email='ping2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ui_sensor_ros2 = lan_control.ui_sensor_ros2:main',
            'ui_sensor_demo = lan_control.ui_sensor_demo:main',
            'ui_sensor_test = lan_control.ui_sensor_test:main',
            'testing = lan_control.testing:main',
            'testing_polyscope = lan_control.testing_polyscope:main',
            'algorithm_demo = lan_control.algorithm_demo:main',
            'pygame_signal = lan_control.pygame_signal:main',
            'keyboard_control = lan_control.keyboard_control:main',
            'sensor_control = lan_control.sensor_control:main',
            'sensor_control_simulation = lan_control.sensor_control_simulation:main',
            'del = lan_control.del:main',


        ],
    },
)
