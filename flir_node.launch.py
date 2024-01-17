from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch import LaunchDescription

example_parameters = {
    'blackfly_s': {
        # set parameters defined in blackfly_s.yaml
        'debug': False,
        'compute_brightness': False,
        'adjust_timestamp': True,
        'dump_node_map': False,
        'gain_auto': 'Continuous',
        # 'pixel_format': 'BayerRG8',
        'exposure_auto': 'Continuous',
        # These are useful for GigE cameras
        # 'device_link_throughput_limit': 380000000,
        'gev_scps_packet_size': 9000,
        'frame_rate_auto': 'Off',
        'frame_rate': 10.0,
        'frame_rate_enable': True,
        'buffer_queue_size': 1,
        'trigger_mode': 'Off',
        'chunk_mode_active': True,
        'chunk_selector_frame_id': 'FrameID',
        'chunk_enable_frame_id': True,
        'chunk_selector_exposure_time': 'ExposureTime',
        'chunk_enable_exposure_time': True,
        'chunk_selector_gain': 'Gain',
        'chunk_enable_gain': True,
        'chunk_selector_timestamp': 'Timestamp',
        'chunk_enable_timestamp': True,
    }
}


def launch_setup(context, *args, **kwargs):
    """Launch camera driver node."""
    parameter_file = LaunchConfig('parameter_file').perform(context)
    camera_type = LaunchConfig('camera_type').perform(context)
    if not parameter_file: parameter_file = PathJoinSubstitution([FindPackageShare('spinnaker_camera_driver'), 'config', camera_type + '.yaml'])
    if camera_type not in example_parameters: raise Exception('no example parameters available for type ' + camera_type)

    node = Node(package='spinnaker_camera_driver',
                executable='camera_driver_node',
                output='screen',
                name=[LaunchConfig('camera_name')],
                parameters=[example_parameters[camera_type], {'parameter_file': parameter_file, 'serial_number': [LaunchConfig('serial')]}],
                remappings=[('~/control', '/exposure_control/control')]
    )

    return [node]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return LaunchDescription([
        LaunchArg('camera_name', default_value=['flir_camera'], description='camera name (ros node name)'),
        LaunchArg('camera_type', default_value='blackfly_s', description='type of camera (blackfly_s, chameleon...)'),
        LaunchArg('serial', default_value="'23422874'", description='FLIR serial number of camera (in quotes!!)'),
        LaunchArg('parameter_file', default_value='',description='path to ros parameter definition file (override camera type)'),
        OpaqueFunction(function=launch_setup)
    ])
