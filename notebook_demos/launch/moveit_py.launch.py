import os
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,Shutdown)

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    hand = LaunchConfiguration("hand")
    servo = LaunchConfiguration("servo")

    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.106.39",
        description="Robot IP address.",
    )

    hand_arg = DeclareLaunchArgument(
        "hand",
        default_value="true",
        description="Whether to use the hand.",
    )

    servo_arg = DeclareLaunchArgument(
        "servo",
        default_value="false",
        description="Whether to use servo.",
    )
    
    
    moveit_config = (
            MoveItConfigsBuilder(robot_name="franka_panda", package_name="moveit_resources_franka_panda_moveit_config")
            .robot_description(file_path=get_package_share_directory("moveit_resources_franka_panda_description") + "/urdf/panda_arm.urdf.xacro", 
                mappings={"robot_ip": robot_ip, "hand": hand})
            .robot_description_semantic("config/franka_panda.srdf")
            .trajectory_execution("config/moveit_controllers.yaml")
            .to_moveit_configs()
            )

    # RViz
    rviz_config = os.path.join(get_package_share_directory('panda_moveit_demos'), 'config', 'panda_default.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Publish TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'source_list': ['franka/joint_states'], 'rate': 30}],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('moveit_resources_franka_panda_moveit_config'),
        'config',
        'ros2_controllers.yaml',
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )

    # Gripper Action Server
    gripper_config = os.path.join(get_package_share_directory('franka_gripper'), 'config',
                                  'franka_gripper_node.yaml')

    gripper = Node(
        package="franka_gripper",
        executable="franka_gripper_node",
        name=['panda_gripper'],
        parameters=[{'robot_ip': robot_ip, 'joint_names': ['panda_finger_joint1','panda_finger_joint2']}, gripper_config],
            )
    
    # Load controllers
    load_controllers = []
    for controller in ['panda_arm_controller', 'panda_gripper', 'joint_state_broadcaster']:
        load_controllers += [
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner {}'.format(controller)],
                shell=True,
                output='screen',
            )
        ]

    notebook_dir = os.path.join(get_package_share_directory("panda_moveit_demos"), "notebooks")
    start_notebook = ExecuteProcess(cmd = ["cd {} && python3 -m notebook --allow-root".format(notebook_dir)], shell = True, output = "screen")
    
    if servo:
        servo_yaml = load_yaml("panda_moveit_demos", "config/servo.yaml")
        servo_params = {"moveit_servo": servo_yaml}

        joy_node = Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
        )

        servo_node = Node(
            package="moveit_servo",
            executable="servo_node_main",
            parameters=[
                servo_params,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
            output="screen",
        )

        return LaunchDescription(
            [
            robot_ip_arg,
            hand_arg,
            servo_arg,
            gripper,
            start_notebook,
            rviz_node,
            static_tf,
            robot_state_publisher,
            joint_state_publisher,
            ros2_control_node,
            joy_node, 
            servo_node,
            ]
            + load_controllers
        )
    else:
        return LaunchDescription(
            [
            robot_ip_arg,
            hand_arg,
            servo_arg,
            gripper,
            start_notebook,
            rviz_node,
            static_tf,
            robot_state_publisher,
            joint_state_publisher,
            ros2_control_node,
            ]
            + load_controllers
        )
