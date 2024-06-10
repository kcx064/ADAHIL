# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    unrealdata_send = Node(
        package="adahil_send_unrealdata",
        executable="send_unrealdata",
		ros_arguments=["--ros-args","--log-level","info"]
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [unrealdata_send])
    # 返回让ROS2根据launch描述执行节点
	# launch_description = LaunchDescription(
    #     [action_robot_01, action_control_01])
    return launch_description
