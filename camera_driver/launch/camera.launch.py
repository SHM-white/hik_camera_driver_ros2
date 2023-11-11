# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    #定义yaml_path为yaml文件路径
    yaml_path = os.path.join(
        get_package_share_directory('camera_driver'),
        '../../../../src/hik_camera_driver_ros2/camera_driver/' #我觉得这有很大问题，但我没有更好的办法
        'yaml',
        'camera.yaml'
    )
    node_01 = Node(
        package="camera_driver",
        executable="camera_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[yaml_path],
        name="camera_driver_node",
        respawn=True
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [node_01]
    )
    # 返回让ROS2根据launch描述执行节点
    return launch_description