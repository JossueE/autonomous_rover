import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


VALID_MODES = ('outdoor', 'indoor')


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in VALID_MODES:
        raise RuntimeError(
            f"Modo inválido: '{mode}'. Opciones: {', '.join(VALID_MODES)}"
        )

    pkg_dir = get_package_share_directory('person_tracker')
    config_file = os.path.join(pkg_dir, 'config', f'person_tracker_params_{mode}.yaml')

    if not os.path.isfile(config_file):
        raise RuntimeError(f"No existe el archivo de parámetros: {config_file}")

    return [
        LogInfo(msg=f'[person_tracker] Modo: {mode.upper()} → {config_file}'),
        Node(
            package='person_tracker',
            executable='person_tracker_node',
            name='person_tracker_node',
            output='screen',
            parameters=[config_file],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='outdoor',
            description='Modo de navegación: outdoor (rápido, exterior) o indoor (suave, laboratorio)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
