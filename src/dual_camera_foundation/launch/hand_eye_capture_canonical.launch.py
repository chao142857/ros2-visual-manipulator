            
"""Canonical parameterized hand-eye capture launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'base_frame': 'base_link', 'tool_frame': 'tool0',
        'camera_frame': 'wrist_d405_color_optical_frame',
        'target_frame': 'wrist_calibration_target_6',
        'target_pose_topic': '/wrist_camera/wrist_d405/calibration/target_pose',
        'output_file': '/tmp/d405_hand_eye_samples.yaml', 'sample_count': '20',
        'overwrite': 'false', 'minimum_translation_change_m': '0.01',
        'minimum_rotation_change_deg': '5.0',
        'max_observation_age_sec': '1.0',
        'future_observation_tolerance_sec': '0.05',
        'tf_lookup_timeout_sec': '0.2',
    }
    arguments = [DeclareLaunchArgument(key, default_value=value)
                 for key, value in defaults.items()]
    integer_parameters = {'sample_count'}
    boolean_parameters = {'overwrite'}
    float_parameters = {
        'minimum_translation_change_m',
        'minimum_rotation_change_deg',
        'max_observation_age_sec',
        'future_observation_tolerance_sec',
        'tf_lookup_timeout_sec',
    }
    parameters = {}
    for key in defaults:
        if key in integer_parameters:
            parameters[key] = ParameterValue(LaunchConfiguration(key), value_type=int)
        elif key in boolean_parameters:
            parameters[key] = ParameterValue(LaunchConfiguration(key), value_type=bool)
        elif key in float_parameters:
            parameters[key] = ParameterValue(LaunchConfiguration(key), value_type=float)
        else:
            parameters[key] = LaunchConfiguration(key)
    arguments.append(Node(
        package='dual_camera_foundation', executable='hand_eye_capture',
        name='hand_eye_capture', output='screen', parameters=[parameters],
    ))
    return LaunchDescription(arguments)

        

