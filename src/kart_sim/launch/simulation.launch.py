"""Master launch file for the kart simulation.

Starts:
1. Gazebo server (headless, initially paused)
2. ros_gz_bridge (topic bridging)
3. Perfect perception node (bypasses YOLO)
4. Cone follower control node
5. Unpause simulation
6. (Optional) Perception pipeline via perception_3d.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_kart_sim = get_package_share_directory("kart_sim")
    world_file = os.path.join(pkg_kart_sim, "worlds", "fs_track.sdf")
    model_path = os.path.join(pkg_kart_sim, "models")

    use_perception_arg = DeclareLaunchArgument(
        "use_perception",
        default_value="false",
        description="Launch the full YOLO perception pipeline (requires weights).",
    )
    use_perfect_perception_arg = DeclareLaunchArgument(
        "use_perfect_perception",
        default_value="true",
        description="Use perfect perception (ground truth from SDF).",
    )

    # --- 1. Gazebo Server (headless) ---
    gazebo_server = ExecuteProcess(
        cmd=[
            "ign", "gazebo", "-s", "-r", "--headless-rendering",
            world_file,
        ],
        output="screen",
        additional_env={
            "IGN_GAZEBO_RESOURCE_PATH": model_path,
        },
    )

    # --- 2. ros_gz_bridge ---
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/fs_track/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/model/kart/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/kart/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/kart/rgbd/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/kart/rgbd/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/kart/rgbd/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
        remappings=[
            ("/kart/rgbd/image", "/zed/zed_node/rgb/image_rect_color"),
            ("/kart/rgbd/depth_image", "/zed/zed_node/depth/depth_registered"),
            ("/kart/rgbd/camera_info", "/zed/zed_node/rgb/camera_info"),
            ("/world/fs_track/clock", "/clock"),
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # --- 3. Perfect perception node ---
    perfect_perception = Node(
        package="kart_sim",
        executable="perfect_perception_node.py",
        name="perfect_perception",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "world_sdf": world_file,
                "output_topic": "/perception/cones_3d",
                "max_range": 20.0,
                "fov_deg": 120.0,
                "publish_rate": 10.0,
                "kart_start_x": 20.0,
                "kart_start_y": 0.0,
                "kart_start_yaw": 1.5708,
            }
        ],
        condition=IfCondition(LaunchConfiguration("use_perfect_perception")),
    )

    # --- 4. Cone follower control node ---
    cone_follower = Node(
        package="kart_sim",
        executable="cone_follower_node.py",
        name="cone_follower",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "detections_topic": "/perception/cones_3d",
                "cmd_vel_topic": "/kart/cmd_vel",
                "max_speed": 2.0,
                "min_speed": 0.5,
                "steering_gain": 1.5,
            }
        ],
    )

    # --- 5. Optional: full perception pipeline ---
    try:
        pkg_perception = get_package_share_directory("kart_perception")
        perception_launch = TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_perception, "launch", "perception_3d.launch.py")
                    ),
                    launch_arguments={
                        "image_topic": "/zed/zed_node/rgb/image_rect_color",
                        "depth_topic": "/zed/zed_node/depth/depth_registered",
                        "camera_info_topic": "/zed/zed_node/rgb/camera_info",
                    }.items(),
                    condition=IfCondition(LaunchConfiguration("use_perception")),
                )
            ],
        )
    except Exception:
        perception_launch = TimerAction(period=1.0, actions=[])

    # --- 7. Cone marker visualization ---
    marker_viz = Node(
        package="kart_perception",
        executable="cone_marker_viz_3d",
        name="cone_marker_viz_3d",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "detections_topic": "/perception/cones_3d",
                "markers_topic": "/perception/cones_3d_markers",
            }
        ],
    )

    return LaunchDescription(
        [
            use_perception_arg,
            use_perfect_perception_arg,
            gazebo_server,
            TimerAction(period=3.0, actions=[bridge]),
            TimerAction(period=5.0, actions=[perfect_perception]),
            TimerAction(period=6.0, actions=[cone_follower]),
            TimerAction(period=5.0, actions=[marker_viz]),
            perception_launch,
        ]
    )
