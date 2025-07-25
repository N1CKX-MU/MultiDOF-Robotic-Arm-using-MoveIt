from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import FindExecutable, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # URDF/XACRO Processing
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("custom_config"), "config", "panda.urdf.xacro"]),
    ])
    robot_description = {"robot_description": robot_description_content}

    # Controller Configuration
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("custom_config"), "config", "ros2_controllers.yaml"
    ])

    # MoveIt Configuration Path
    moveit_config = PathJoinSubstitution([
        FindPackageShare("custom_config"), "config"
    ])

    # Controller Manager Node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )

    return LaunchDescription([
        controller_manager_node,

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),

        # MoveGroup Node
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                robot_description,
                {"use_sim_time": False},
                PathJoinSubstitution([moveit_config, "kinematics.yaml"]),
                PathJoinSubstitution([moveit_config, "moveit_controllers.yaml"]),
            ],
        ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("custom_config"), "config", "moveit.rviz"
            ])],
            output="screen",
        ),

        # Static TF Publisher
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0.0", "0", "0", "0", "world", "panda_link0"],
        ),

        # Spawn Controllers after ros2_control_node starts
        RegisterEventHandler(
            OnProcessStart(
                target_action=controller_manager_node,
                on_start=[
                    TimerAction(
                        period=5.0,
                        actions=[
                            ExecuteProcess(
                                cmd=["ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager"],
                                shell=True,
                                output="screen",
                            ),
                            ExecuteProcess(
                                cmd=["ros2 run controller_manager spawner panda_arm_hand_controller --controller-manager /controller_manager"],
                                shell=True,
                                output="screen",
                            )
                        ],
                    )
                ],
            )
        ),
    ])
