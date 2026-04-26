#!/usr/bin/env python3
"""
Launch file to start the OAK-D S2 camera using depthai_ros_driver.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterFile


def generate_launch_description():
    """Declare arguments and defer camera node setup to launch_setup via OpaqueFunction."""
    declared_arguments = [
        DeclareLaunchArgument(
            "pointcloud",
            default_value="false",
            description="Use oakd_vio_pcl.yaml (with RGBD point cloud) instead of oakd_vio.yaml.",
        ),
    ]

    def launch_setup(context, *_args, **_kwargs):
        """Select oakd_vio.yaml or oakd_vio_pcl.yaml based on the pointcloud arg and start the OAK-D composable node container."""
        log_level = "info"
        if context.environment.get("DEPTHAI_DEBUG") == "1":
            log_level = "debug"

        pointcloud = LaunchConfiguration("pointcloud").perform(context) == "true"
        config_file = "oakd_vio_pcl.yaml" if pointcloud else "oakd_vio.yaml"
        params_file = ParameterFile(
            os.path.join(
                get_package_share_directory("lekiwi_bringup"),
                "config",
                config_file,
            ),
            allow_substs=True,
        )

        return [
            ComposableNodeContainer(
                name="oak_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::Driver",
                        name="oak",
                        parameters=[params_file, {
                            "driver": {
                                "i_tf_parent_frame": "tilt_link",
                                "i_tf_camera_model": "OAK-D-S2",
                                "i_tf_base_frame": "oak_link",
                            }
                        }],
                    )
                ],
                arguments=["--ros-args", "--log-level", log_level],
                output="both",
            ),
        ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
