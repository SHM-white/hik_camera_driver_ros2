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
        'yaml',
        'camera.yaml'
    )
    node_01 = Node(
        package="camera_driver",
        executable="camera_driver_node",
        output="screen",
        emulate_tty=True, #用来保证printf可以打印
        parameters=[yaml_path],
        name="camera_driver_node",
        respawn=False,
        namespace="infantry" #一定要注明命名空间，否则无法读入参数
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [node_01]
    )
    # 返回让ROS2根据launch描述执行节点
    return launch_description
