from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    px4_path = os.path.expanduser('~/PX4-Autopilot')

    actions = []

    actions.append(ExecuteProcess(
        cmd=['./Tools/simulation/gz/simulation-gazebo'],
        cwd=px4_path,
        output='screen'
    ))

    actions.append(ExecuteProcess(
        cmd=['micro-xrce-dds-agent', 'udp4', '-p', '8888'],
        output='screen'
    ))

    # --- Dron 0: LIDER ---
    actions.append(TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['./build/px4_sitl_default/bin/px4', '-i', '0'],
                cwd=px4_path,
                env={
                    **os.environ,
                    'PX4_SYS_AUTOSTART': '4001',
                    'PX4_GZ_MODEL_POSE': '0,0',
                    'PX4_SIMULATOR': 'gz',
                    'PX4_GZ_MODEL': 'x500',
                },
                output='screen'
            )
        ]
    ))

    # --- Dron 1: PODĄŻAJĄCY ---
    actions.append(TimerAction(
        period=18.0,
        actions=[
            ExecuteProcess(
                cmd=['./build/px4_sitl_default/bin/px4', '-i', '1'],
                cwd=px4_path,
                env={
                    **os.environ,
                    'PX4_SYS_AUTOSTART': '4001',
                    'PX4_GZ_MODEL_POSE': '-3,0',
                    'PX4_SIMULATOR': 'gz',
                    'PX4_GZ_MODEL': 'x500',
                },
                output='screen'
            )
        ]
    ))
  
    # --- Węzeł LIDERA ---
    actions.append(TimerAction(
        period=35.0,
        actions=[
            Node(
                package='swarm_control',
                executable='offboard_control',
                namespace='',
                parameters=[{
                    'role': 'leader',
                    'target_system': 1,
                    'waypoints': [5.0, 0.0,
                                  10.0, 5.0,
                                  15.0, 0.0,
                                  10.0, -5.0],
                    'cruise_altitude': -9.0,
                    'land_hold_seconds': 5.0,
                }],
                output='screen'
            )
        ]
    ))

    # --- Węzeł PODĄŻAJĄCEGO ---
    actions.append(TimerAction(
        period=43.0,
        actions=[
            Node(
                package='swarm_control',
                executable='offboard_control',
                namespace='px4_1',
                parameters=[{
                    'role': 'follower',
                    'target_system': 2,
                    'leader_position_topic': '/fmu/out/vehicle_local_position',
                    'offset_x':  0.0,
                    'offset_y':  3.0,
                    'offset_z':  -4.0,
                }],
                output='screen'
            )
        ]
    ))
    
    return LaunchDescription(actions)
