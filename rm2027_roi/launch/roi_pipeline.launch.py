from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("rm2027_roi")
    config = lambda name: os.path.join(share, "config", name)

    detector = Node(
        package="rm2027_roi",
        executable="roi_detector_node",
        name="roi_detector",
        parameters=[config("detector.yaml"), config("pipeline.yaml")],
        remappings=[("image_raw", "/camera/image_raw")],
    )
    estimator = Node(
        package="rm2027_roi",
        executable="roi_estimator_node",
        name="roi_estimator",
        parameters=[config("kalman.yaml"), config("pipeline.yaml")],
    )
    crop = Node(
        package="rm2027_roi",
        executable="roi_crop_node",
        name="roi_crop",
        parameters=[config("coding.yaml"), config("pipeline.yaml")],
        remappings=[("image_raw", "/camera/image_raw")],
    )
    return LaunchDescription([detector, estimator, crop])
