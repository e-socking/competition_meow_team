from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument  
from launch.substitutions import LaunchConfiguration  

# 兼容ROS 2的条件判断类（实现evaluate方法）
class LaunchConfigurationEquals:
    def __init__(self, launch_config_name, value):
        self.launch_config_name = launch_config_name
        self.value = value

    def evaluate(self, context):
        config_value = context.launch_configurations.get(self.launch_config_name)
        return config_value == self.value

def generate_launch_description():
    # 1. 声明启动参数（4空格缩进，无格式错误）
    declare_node_arg = DeclareLaunchArgument(
        name='node_to_launch',
        default_value='armor_node',
        description='指定要启动的节点：armor_node / sphere_node / rect_node'
    )

    # 2. 定义节点（删除重复condition，补全语法）
    a = Node(
        package="team_meow_challenge",
        executable="armor_node",
        condition=LaunchConfigurationEquals('node_to_launch', 'armor_node')  # 唯一condition
    )
    b = Node(
        package="team_meow_challenge",
        executable="rect_node",
        condition=LaunchConfigurationEquals('node_to_launch', 'rect_node')
    )
    c = Node(
        package="team_meow_challenge",
        executable="sphere_node",
        condition=LaunchConfigurationEquals('node_to_launch', 'sphere_node')
    )
    
    # 3. 组装返回（无语法错误）
    return LaunchDescription([
        declare_node_arg,  
        a,       
        b,        
        c     
    ])

