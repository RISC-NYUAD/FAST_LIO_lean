#!/usr/bin/env python3

import ament_index_python
import launch
import launch_ros
import os
import yaml


def generate_launch_description():

    # ##### Velodyne

    # Driver extracts message packets
    velodyne_driver_config_path = os.path.join(
        ament_index_python.packages.get_package_share_directory("velodyne_driver"),
        "config",
        "VLP16-velodyne_driver_node-params.yaml",
    )
    with open(velodyne_driver_config_path, "r") as f:
        velodyne_driver_params = yaml.safe_load(f)["velodyne_driver_node"][
            "ros__parameters"
        ]

    velodyne_driver_node = launch_ros.descriptions.ComposableNode(
        package="velodyne_driver",
        plugin="velodyne_driver::VelodyneDriver",
        name="velodyne_driver_node",
        parameters=[velodyne_driver_params],
    )

    # Converter converts the packets into a pointcloud2
    velodyne_transform_config_path = os.path.join(
        ament_index_python.packages.get_package_share_directory("velodyne_pointcloud"),
        "config",
        "VLP16-velodyne_transform_node-params.yaml",
    )
    with open(velodyne_transform_config_path, "r") as f:
        velodyne_transform_params = yaml.safe_load(f)["velodyne_transform_node"][
            "ros__parameters"
        ]

    velodyne_transform_params["calibration"] = os.path.join(
        ament_index_python.packages.get_package_share_directory("velodyne_pointcloud"),
        "params",
        "VLP16db.yaml",
    )

    velodyne_transform_node = launch_ros.descriptions.ComposableNode(
        package="velodyne_pointcloud",
        plugin="velodyne_pointcloud::Transform",
        name="velodyne_transform_node",
        parameters=[velodyne_transform_params],
    )

    velodyne_container = launch_ros.actions.ComposableNodeContainer(
        name="velodyne_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[velodyne_driver_node, velodyne_transform_node],
        output="both",
        emulate_tty=True,
    )

    # ##### Realsense & IMU filter
    realsense_launch_description = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [
                launch.substitutions.PathJoinSubstitution(
                    [
                        launch_ros.substitutions.FindPackageShare("realsense2_camera"),
                        "launch",
                        "rs_launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "initial_reset": "true",
            "depth_module.profile": "848,480,90",
            "enable_depth": "true",
            "rgb_camera.profile": "848,480,30",
            "enable_color": "true",
            "enable_infra": "false",
            "align_depth.enable": "true",
            "enable_accel": "true",
            "enable_gyro": "true",
            "gyro_fps": "200",
            "accel_fps": "200",
            "unite_imu_method": "2",
            "publish_tf": "true",
        }.items(),
    )

    filter_node = launch_ros.actions.Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        output="both",
        parameters=[
            {
                "use_mag": False,
                "publish_tf": False,
                "fixed_frame": "odom",
                "world_frame": "enu",
            }
        ],
        remappings=[
            ("imu/data_raw", "camera/imu"),
            ("imu/data", "camera/imu_filtered"),
        ],
        emulate_tty=True,
    )

    # ##### TF
    static_tf_node = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "-0.09280285",
            "-0.06356535",
            "0.09051928",
            "-0.00467982",
            "0.04877741",
            "-0.01023238",
            "camera_link",
            "velodyne",
        ],
        output="both",
        emulate_tty=True,
    )

    # # ##### Mapping
    # fast_lio_config_path = os.path.join(
    #     ament_index_python.packages.get_package_share_directory("fast_lio"),
    #     "config",
    #     "velodyne.yaml",
    # )
    # fast_lio_node = Node(
    #     package="fast_lio",
    #     executable="fastlio_mapping",
    #     parameters=[config_path, {"use_sim_time": use_sim_time}],
    #     output="screen",
    # )

    return launch.LaunchDescription(
        # [velodyne_container, realsense_launch_description, filter_node, static_tf_node]
        [velodyne_container, realsense_launch_description, static_tf_node]
    )
