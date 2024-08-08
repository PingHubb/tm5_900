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
            'ui_sensor_1 = lan_control.ui_sensor_1:main',
            'ui_sensor_all = lan_control.ui_sensor_all:main',
            'testing_polyscope = lan_control.testing_polyscope:main',
            'testing = lan_control.testing:main',
            'testing_2 = lan_control.testing_2:main',
            'algorithm_demo = lan_control.algorithm_demo:main',
            'pygame_signal = lan_control.pygame_signal:main',
            'keyboard_control = lan_control.OLD.keyboard_control:main',
            'sensor_control = lan_control.sensor_control:main',
            'sensor_control_simulation = lan_control.sensor_control_simulation:main',
            'gg_ros = lan_control.gg_ros:main',
            'Command_Send_Sub = lan_control.Command_Send_Sub:main',
            'pvt_command = lan_control.pvt_command:main',
            'QT5 = lan_control.QT5:main',

        ],
    },
)
