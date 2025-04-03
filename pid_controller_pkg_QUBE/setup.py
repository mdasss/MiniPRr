from setuptools import find_packages, setup

package_name = 'pid_controller_pkg_QUBE'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', [
        'urdf/qube.urdf.xacro',
        'urdf/qube.macro.xacro',
        'urdf/controlled_qube.urdf.xacro'
        ]),
        
        ('share/' + package_name + '/launch', [
        'launch/launch.py',
        'launch/bringup.launch.py',
        'launch/view_qube.launch.py']),
        
        ('share/' + package_name + '/config', [
        'config/parameters.yaml',
        'config/configuration_rviz.rviz',
        'config/controllers.yaml'
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drinkalotofwater',
    maintainer_email='drinkalotofwater@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [ 
        'pid_controller_node_QUBE = pid_controller_QUBE.pid_controller_node_QUBE:main',
    ],
},
)
