from launch import LaunchDescription
from launch_ros.actions import Node
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
    output="screen",
    parameters=[params]
  )

  bag_exec = ExecuteProcess(
    cmd=["ros2", "bag", "play", "/data/Kitti/raw/kitti_2011_09_26_drive_0014_synced"]
  )

  return LaunchDescription([
    stereo_slam_node,
    bag_exec
  ])