import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    fishbot_description_dir = get_package_share_directory('fishbot_description')
    
    # URDF文件路径
    urdf_path = PathJoinSubstitution([
        fishbot_description_dir,
        'urdf',
        'fishbot.urdf'
    ])
    
    # 读取URDF文件内容
    try:
        with open(urdf_path.perform(None), 'r') as f:
            robot_description_content = f.read()
    except FileNotFoundError:
        raise RuntimeError(f"URDF文件未找到: {urdf_path.perform(None)}")
    
    # robot_state_publisher节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='fishbot',
        parameters=[{'robot_description': robot_description_content}],
        output='screen'
    )
    
    # 里程计到TF的转换节点
    odom2tf = Node(
        package='fishbot_bringup',
        executable='odom2tf',
        name='odom2tf',
        namespace='fishbot',
        output='screen',
        parameters=[{
            'publish_frequency': 30.0,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint'
        }]
    )
    
    # 串口通信节点（保留原注释内容）
    stm32_bridge = launch_ros.actions.Node(
        package='demo_cpp_pkg',
        executable='ros2_cpp_node',
        name='ros2_cpp_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'baudrate': 115200,
            'base_frame': 'base_footprint'
        }],
        remappings=[
            ('/joint_states', '/stm32/joint_states')
        ],
        output='screen',
        on_exit=launch.actions.Shutdown()
    )
        
    # 静态TF转换（base_link到footprint）
    base_to_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_footprint',
        namespace='fishbot',
        arguments=['0', '0', '0.06', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen'
    )
    
    # 获取rplidar_ros包路径
    rplidar_ros_dir = get_package_share_directory('rplidar_ros')

    # 使用rplidar_a1_launch.py启动雷达节点（修正：使用GroupAction确保命名空间生效）
    rplidar_node = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [rplidar_ros_dir, '/launch', '/rplidar_a1_launch.py']
                ),
                launch_arguments={
                    'namespace': 'fishbot',  # 确保雷达节点也在fishbot命名空间下
                    'serial_port': '/dev/ttyUSB1',
                    'frame_id': 'laser_link',
                    'angle_compensate': 'true',
                    'scan_mode': 'Standard',
                }.items()
            )
        ]
    )
    
    # 警告：如果你手动启动了多个tf2_echo节点，需要停止它们或为它们指定唯一名称
    # 例如：ros2 run tf2_ros tf2_echo __name:=my_tf2_echo source_frame target_frame
    
    return launch.LaunchDescription([
        base_to_footprint,
        robot_state_publisher,
        odom2tf,
        rplidar_node,
        stm32_bridge,
        
        launch.actions.LogInfo(
            msg=f"FishBot系统启动成功，已加载URDF: {urdf_path.perform(None)}"
        )
    ])