from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
  params = join(
    get_package_share_directory("stereo_slam"), "params", "camera.yaml"
  )

  stereo_slam_node = Node(
    package="stereo_slam",
    executable="stereo_slam_node",
    name="stereo_slam_node",
    parameters=[params]
  )

  cam_to_base_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='camera2base_tf',
    arguments = ['--x', '0.0', '--y', '0.0', '--z', '0', '--roll', '1.57', '--pitch', '-1.57', '--yaw', '0.0', '--frame-id', 'base_link', '--child-frame-id', 'camera_link']
  )

  gps_to_base_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='gps2base_tf',
    arguments = ['--x', '-1.08', '--y', '0.32', '--z', '-0.72', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'base_link', '--child-frame-id', 'gps_link']
  )

  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    arguments=["-d", join(get_package_share_directory("stereo_slam"), "rviz/", "stereo_slam.rviz")]
  )

  bag_exec = ExecuteProcess(
    cmd=["ros2", "bag", "play", "-r", "1.0", "/data/kitti/raw/2011_09_29_drive_0071_sync_bag" , "--clock"]
  )

  trajectory_server_gps_node=Node(
    package="trajectory_server",
    executable="trajectory_server_node",
    name="trajectory_server_node",
    namespace = "oxts",
    parameters=[{
      "target_frame_name": "odom",
      "source_frame_name": "gps_link",
      "trajectory_update_rate": 10.0,
      "trajectory_publish_rate": 10.0
    }]
  )

  trajectory_server_camera_node=Node(
    package="trajectory_server",
    executable="trajectory_server_node",
    name="trajectory_server_node",
    namespace = "camera",
    parameters=[{
      "target_frame_name": "odom",
      "source_frame_name": "camera_link",
      "trajectory_update_rate": 10.0,
      "trajectory_publish_rate": 10.0
    }]
  )

  return LaunchDescription([
    SetParameter(name="use_sim_time", value=True),
    stereo_slam_node,
    cam_to_base_tf,
    gps_to_base_tf,
    rviz_node,
    bag_exec,
    trajectory_server_gps_node,
    trajectory_server_camera_node
  ])
