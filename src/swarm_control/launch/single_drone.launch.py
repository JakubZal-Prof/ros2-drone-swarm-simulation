from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    px4_path = os.path.expanduser('~/PX4-Autopilot')

    drones = [
        {"id": 0,"sys_id": 1, "ns": "",      "pose": "0,0",  "target": (0.0, 0.0, -5.0)},
        {"id": 1,"sys_id": 2, "ns": "px4_1", "pose": "3,0",  "target": (3.0, 3.0, -5.0)},
    ]

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

    for i, drone in enumerate(drones):
        actions.append(TimerAction(
            period=10.0 + i * 8.0,
            actions=[
                ExecuteProcess(
                    cmd=['./build/px4_sitl_default/bin/px4', '-i', str(drone["id"])],
                    cwd=px4_path,
                    env={
                        **os.environ,
                        'PX4_SYS_AUTOSTART': '4001',
                        'PX4_GZ_MODEL_POSE': drone["pose"],
                        'PX4_SIMULATOR': 'gz',
                        'PX4_GZ_MODEL': 'x500',
                    },
                    output='screen'
                )
            ]
        ))

    for i, drone in enumerate(drones):
        actions.append(TimerAction(
            period=35.0 + i * 8.0,
            actions=[
                Node(
                    package='swarm_control',
                    executable='offboard_control',
                    namespace=drone["ns"],
                    parameters=[{
                        'target_x': drone["target"][0],
                        'target_y': drone["target"][1],
                        'target_z': drone["target"][2],
			'target_system': drone["sys_id"],
                    }],
                    output='screen'
                )
            ]
        ))

    return LaunchDescription(actions)
