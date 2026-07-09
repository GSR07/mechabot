from setuptools import find_packages, setup

package_name = 'mechabot_scripts'

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
    maintainer='gsr',
    maintainer_email='ganeshsreddy1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'read_lidar = mechabot_scripts.read_lidar:main',
            'read_imu = mechabot_scripts.read_imu:main',
            'read_camera = mechabot_scripts.read_camera:main',
            'detect_marker = mechabot_scripts.detect_marker:main',
            'maze_solver = mechabot_scripts.maze_solver:main',
            'auto_dock_undock = mechabot_scripts.auto_dock_undock:main',
            'docling_with_patrolling = mechabot_scripts.docling_with_patrolling:main',
            'battery_auto_dock=  mechabot_scripts.auto_docking_with_battery:main',
            'cleaning = mechabot_scripts.Cleaning_with_battery:main',
            'rl_nav = mechabot_scripts.rl_docking:main',
        ],
    },
)
