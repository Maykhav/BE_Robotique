from setuptools import setup

package_name = 'mearm_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', [
            'urdf/robot.urdf.xacro',
            'urdf/ball.urdf.xacro',
            'urdf/robot.urdf']),
        ('share/' + package_name + '/config', [
            'config/joint_positions_sets.yaml']),
        ('share/' + package_name + '/rviz', [
            'rviz/urdf.rviz']),
        ('share/' + package_name + '/launch', [
            'launch/display.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmed',
    maintainer_email='ahmed@example.com',
    description='MeArm model package for ROS2 with URDF, RViz and Python nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_publisher = mearm_model.ball_publisher:main',
            'filter = mearm_model.filter:main',
            'move = mearm_model.move:main',
        ],
    },
)
