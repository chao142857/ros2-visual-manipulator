"""Launch tag localization and one explicit camera calibration session."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build calibration mode without a competing runtime static TF."""
    default_extrinsics_file = PathJoinSubstitution([
        FindPackageShare('camera_perception'),
        'config',
        'camera_extrinsics.yaml',
    ])

    start_tag_localizer = LaunchConfiguration('start_tag_localizer')
    target_tag_id = LaunchConfiguration('target_tag_id')
    sample_count = LaunchConfiguration('sample_count')
    save_calibration = LaunchConfiguration('save_calibration')
    overwrite_calibration = LaunchConfiguration('overwrite_calibration')
    calibration_file = LaunchConfiguration('calibration_file')

    tag_localizer = Node(
        package='camera_perception',
        executable='tag_localizer_node',
        name='tag_localizer_node',
        output='screen',
        condition=IfCondition(start_tag_localizer),
        parameters=[{
            'target_tag_id': ParameterValue(target_tag_id, value_type=int),
        }],
    )

    calibrator = Node(
        package='camera_perception',
        executable='camera_extrinsic_calibrator_node',
        name='camera_extrinsic_calibrator_node',
        output='screen',
        parameters=[{
            'sample_count': ParameterValue(sample_count, value_type=int),
            'save_extrinsics': ParameterValue(
                save_calibration,
                value_type=bool,
            ),
            'overwrite_existing': ParameterValue(
                overwrite_calibration,
                value_type=bool,
            ),
            'extrinsics_file': calibration_file,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_tag_localizer',
            default_value='true',
            description=(
                'Start the existing RGB-D AprilTag localizer needed for tag_0.'
            ),
        ),
        DeclareLaunchArgument(
            'target_tag_id',
            default_value='6',
            description=(
                'Existing localizer target ID; all known tag TFs are still '
                'broadcast, including calibration tag_0.'
            ),
        ),
        DeclareLaunchArgument(
            'sample_count',
            default_value='60',
            description='Number of fresh tag_0 TF observations to average.',
        ),
        DeclareLaunchArgument(
            'save_calibration',
            default_value='false',
            description=(
                'Explicitly save the result; false keeps calibration '
                'print-only.'
            ),
        ),
        DeclareLaunchArgument(
            'overwrite_calibration',
            default_value='false',
            description=(
                'Allow replacement of an existing calibration file. This is '
                'only honored when save_calibration is also true.'
            ),
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=default_extrinsics_file,
            description='ROS parameter YAML written by explicit calibration.',
        ),
        tag_localizer,
        calibrator,
    ])
