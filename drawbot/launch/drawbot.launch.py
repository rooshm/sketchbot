from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
  # Launch cv_pipeline_service
  cv_pipeline_service = Node(
    package='cv_pipeline',
    executable='cv_pipeline_service',
    name='cv_pipeline_service',
    output='screen',
  )

  # Launch svg2path_service
  svg2path_service = Node(
    package='pathgen',
    executable='svg2path_service',
    name='svg2path_service',
    output='screen',
  )

  # Include controller launch file
  sketch_controller = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([FindPackageShare('sketch_controller'), 'launch', 'move_group_interface_tutorial.launch.py']),
    ),
    # TODO: add arguments?
  )

  # Launch realsense2 camera
  realsense2 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']),
    ),
    # launch_arguments={
    #   'align_depth': 'true',
    #   'enable_pointcloud': 'true',
    #   'filters': 'pointcloud',
    #   'tf_publish_rate': '0.0',
    # }.items(),
  )

  # Launch drawbot_service
  drawbot_service = Node(
    package='drawbot',
    executable='drawbot_service',
    name='drawbot_service',
    output='screen',
  )

  return LaunchDescription([
    cv_pipeline_service,
    svg2path_service,
    sketch_controller,
    realsense2,
    drawbot_service,
  ])
