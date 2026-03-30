"""
tidy.launch.py — Single launch entry point for the entire simulation.

Usage:
    ros2 launch tidybot_sim tidy.launch.py
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    ROS 2 launch files must define this function.
    It returns a LaunchDescription containing all processes to start.
    """

    # ── PACKAGE PATHS ──────────────
    # get_package_share_directory() resolves to the installed share directory:
    #   ~/tidybot_ws/install/tidybot_sim/share/tidybot_sim/
    # This is why setup.py's data_files must install everything there.
    pkg = get_package_share_directory('tidybot_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path  = os.path.join(pkg, 'worlds', 'home.sdf')
    xacro_path  = os.path.join(pkg, 'description', 'tidybot.urdf.xacro')
    bridge_path = os.path.join(pkg, 'config', 'bridge.yaml')

    set_sw_render  = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    set_gl_version = SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3')

    # ── LAUNCH ARGUMENTS ───────────
    # These let the reviewer override defaults without editing source:
    #   ros2 launch tidybot_sim tidy.launch.py enable_pickup:=false
    declare_enable_pickup = DeclareLaunchArgument(
        'enable_pickup',
        default_value='true',
        description='Enable pick-and-place pipeline (arm_controller node)',
    )
    declare_robot_x = DeclareLaunchArgument(
        'robot_x', default_value='0.5',
        description='Robot spawn X position',
    )
    declare_robot_y = DeclareLaunchArgument(
        'robot_y', default_value='0.0',
        description='Robot spawn Y position',
    )

    enable_pickup = LaunchConfiguration('enable_pickup')
    robot_x       = LaunchConfiguration('robot_x')
    robot_y       = LaunchConfiguration('robot_y')

    # ── PROCESS XACRO → URDF STRING 
    # xacro.process_file() runs the xacro preprocessor, expanding all
    # macros and ${...} expressions, producing a plain URDF XML string.
    # We pass this string to robot_state_publisher AND use it to spawn the
    # robot (via /robot_description topic).
    robot_description_content = xacro.process_file(xacro_path).toxml()
    robot_description_param   = {'robot_description': robot_description_content}

    # ── 1. IGNITION GAZEBO 
    # We include ros_gz_sim's built-in launch file, which:
    #   - Sets up GZ_SIM_RESOURCE_PATH from all installed ROS packages
    #   - Runs: ruby ign gazebo <gz_args> --force-version 6
    # gz_args:
    #   -r  = run immediately (don't pause at start)
    #   world_path = our SDF world file (absolute path)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}',
            'gz_version': '6',
            'on_exit_shutdown': 'true',  # kill ROS when Gazebo closes
        }.items(),
    )

    # ── 2. ROBOT STATE PUBLISHER ───
    # Reads robot_description (URDF XML string) and publishes TF transforms
    # for every fixed joint in the URDF tree (e.g. base_link → wheel_link).
    # For moving joints (wheels, arm joints), it reads /joint_states to get
    # the current angles, then computes the transform.
    # use_sim_time: true → uses /clock from Gazebo instead of wall time.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description_param,
            {'use_sim_time': True},
        ],
    )

    # ── 3. ROS ↔ GAZEBO BRIDGE ─────
    # parameter_bridge reads bridge.yaml and creates one translation channel
    # per entry. It must be running before any ROS node tries to subscribe
    # to a Gazebo-sourced topic (like /odom or /scan).
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[
            {'config_file': bridge_path},
            {'use_sim_time': True},
        ],
    )

    # ── 4. SPAWN ROBOT (delayed 3s) ─
    # 'ros_gz_sim create' sends a gRPC request to Gazebo to spawn a model.
    # It reads the URDF from /robot_description topic (published by RSP above).
    # -z 0.13: base_link spawns at z=0.13 above ground, which puts wheels
    # exactly at z=0.08 (wheel radius), resting on the ground plane.
    #
    # We delay 3 seconds because Gazebo needs time to:
    #   1. Load the world SDF and initialize physics
    #   2. Start the UserCommands plugin (which handles spawn requests)
    # Spawning before this is ready causes silent failure (no error, no robot).
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_tidybot',
                output='screen',
                arguments=[
                    '-name',  'tidybot',
                    '-topic', '/robot_description',  # read from RSP's topic
                    '-x',     robot_x,
                    '-y',     robot_y,
                    '-z',     '0.13',
                    '-R',     '0',   # roll
                    '-P',     '0',   # pitch
                    '-Y',     '0',   # yaw
                ],
            )
        ],
    )

    # ── 5. APPLICATION NODES (delayed 6s) 
    # These are our behavior nodes. They need:
    #   a) Robot spawned (so /joint_states, /odom exist)
    #   b) Bridge running (so they can subscribe to bridged topics)
    # 6 seconds gives the spawn + bridge time to settle.
    #
    # Phase 4: logger (full CSV) + navigator (waypoint state machine)
    # Phase 5: uncomment arm_controller
    logger_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='tidybot_sim',
                executable='logger',
                name='logger',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ],
    )

    navigator_node = TimerAction(
        period=6.0,
        actions=[Node(
            package='tidybot_sim',
            executable='navigator',
            name='navigator',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )],
    )

    arm_controller_node = TimerAction(
        period=6.0,
        actions=[Node(
            package='tidybot_sim',
            executable='arm_controller',
            name='arm_controller',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )],
    )

    # ── ASSEMBLE LAUNCH DESCRIPTION 
    # All declared arguments must appear here.
    # Order matters for readability but not execution (ROS launches things
    # in parallel unless there's a TimerAction or event dependency).
    return LaunchDescription([
        declare_enable_pickup,
        declare_robot_x,
        declare_robot_y,

        set_sw_render,
        set_gl_version,
        gz_sim,               # starts immediately
        robot_state_publisher, # starts immediately
        bridge,               # starts immediately
        spawn_robot,          # runs after 3s
        logger_node,          # runs after 6s (Phase 4: full CSV logger)
        navigator_node,       # runs after 6s (Phase 4: waypoint navigator)
        arm_controller_node,  # runs after 6s (Phase 5: pick-and-place controller)
    ])
