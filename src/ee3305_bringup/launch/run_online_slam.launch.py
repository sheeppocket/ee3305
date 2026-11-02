"""Start from no prior map; explore toward a goal with the student algorithms."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('ee3305_bringup')
    args = [DeclareLaunchArgument(name, default_value=value) for name, value in [
        ('x', '-2.0'), ('y', '-0.5'), ('model', 'burger'),
        ('world', 'turtlebot3_house'), ('headless', 'False'), ('libgl', 'False'),
        ('rviz', 'True'), ('algorithm', 'dijkstra')]]
    args.append(DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
        [share, 'params', 'online_slam.yaml'])))
    world = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
        [share, 'launch', 'sim_world.launch.py'])), launch_arguments={
            **{key: LaunchConfiguration(key) for key in
               ['x', 'y', 'model', 'world', 'headless', 'libgl']},
            'use_sim_time': 'true'}.items())
    slam = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py'])),
        launch_arguments={'use_sim_time': 'true', 'use_rviz': 'false',
                          'resolution': '0.05', 'publish_period_sec': '0.5'}.items())
    params = [LaunchConfiguration('params_file'), {'use_sim_time': True}]
    nodes = [Node(package='ee3305_py', executable=executable, name=executable,
                  output='screen', parameters=params)
             for executable in ['slam_pose', 'online_navigator', 'online_safety']]
    nodes += [Node(package='ee3305_py', executable='online_planner', name='planner',
                   output='screen', parameters=params + [{'algorithm': LaunchConfiguration('algorithm')}]),
              Node(package='ee3305_py', executable='online_controller', name='controller',
                   output='screen', parameters=params,
                   remappings=[('odom', 'slam_pose'), ('cmd_vel', 'cmd_vel_unchecked')]),
              Node(package='rviz2', executable='rviz2', output='screen',
                   condition=IfCondition(LaunchConfiguration('rviz')),
                   parameters=[{'use_sim_time': True}], arguments=['-d',
                   PathJoinSubstitution([share, 'rviz', 'project.rviz'])])]
    # Deliberately no map loader, static map→odom TF, Nav2 controller, or
    # original behaviour node. Cartographer owns map→odom; OnlineNavigator
    # owns path requests; OnlineSafety is the sole final cmd_vel publisher.
    return LaunchDescription(args + [world, slam] + nodes)
