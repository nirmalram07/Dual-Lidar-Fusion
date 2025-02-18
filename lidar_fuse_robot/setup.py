from setuptools import find_packages, setup

package_name = 'lidar_fuse_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/robot_model.xacro', 'urdf/lidar_bot.urdf.xacro', 'urdf/gazebo_control.xacro', 'urdf/lidar.xacro']),
        ('share/' + package_name + '/config', ['config/gz_bridge.yaml']),
        ('share/' + package_name + '/worlds', ['worlds/empty.world']),
        ('share/' + package_name + '/launch', ['launch/urdf_model.launch.py', 'launch/diff_bot.launch.py', 'launch/diff_robot.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nirmal',
    maintainer_email='nirmalcgvfx@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
