from setuptools import find_packages, setup

package_name = 'ee3305_py'

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
    maintainer='Lai Yan Kai',
    maintainer_email='lai.yankai@gmail.com',
    description='https://www.youtube.com/watch?v=dQw4w9WgXcQ',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior = ee3305_py.behavior:main',
            'planner = ee3305_py.planner:main',
            'controller = ee3305_py.controller:main',
            'online_navigator = ee3305_py.online_slam:main',
            'slam_pose = ee3305_py.online_slam:pose_main',
            'online_safety = ee3305_py.online_safety:main',
            'online_planner = ee3305_py.online_adapters:planner_main',
            'online_controller = ee3305_py.online_adapters:controller_main',
        ],
    },
)
