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

    start_enabled = LaunchConfiguration('start_enabled').perform(context)

    pkg_dir = get_package_share_directory('person_tracker')
    config_file = os.path.join(pkg_dir, 'config', f'person_tracker_params_{mode}.yaml')
    indoor_file = os.path.join(pkg_dir, 'config', 'person_tracker_params_indoor.yaml')
    outdoor_file = os.path.join(pkg_dir, 'config', 'person_tracker_params_outdoor.yaml')

    if not os.path.isfile(config_file):
        raise RuntimeError(f"No existe el archivo de parámetros: {config_file}")

    return [
        LogInfo(msg=f'[person_tracker] Modo: {mode.upper()} → {config_file}'),
        Node(
            package='person_tracker',
            executable='person_tracker_node',
            name='person_tracker_node',
            output='screen',
            # Base profile YAML first, then overrides so the indoor/outdoor files
            # are known to the node for live /person_tracker/profile switching and
            # start_enabled reflects whether a supervisor (rover_bt) drives it.
            parameters=[
                config_file,
                {
                    'params_indoor_file': indoor_file,
                    'params_outdoor_file': outdoor_file,
                    'start_enabled': start_enabled.lower() in ('true', '1'),
                },
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='indoor',
            description='Modo de navegación: outdoor (rápido, exterior) o indoor (suave, laboratorio)',
        ),
        DeclareLaunchArgument(
            'start_enabled',
            default_value='true',
            description='Si arranca siguiendo de inmediato (true) o esperando la '
                        'señal /person_tracker/enable de rover_bt (false)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
