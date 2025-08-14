#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    imu_topic = LaunchConfiguration('imu_topic')
    imu_qos   = LaunchConfiguration('imu_qos')
    mode      = LaunchConfiguration('mode')
    turn_lin  = LaunchConfiguration('turn_lin')

    return LaunchDescription([
        # ── 인자
        DeclareLaunchArgument('imu_topic', default_value='/imu'),           # stella_ahrs 기본
        DeclareLaunchArgument('imu_qos',   default_value='reliable'),       # stella_ahrs는 보통 RELIABLE
        DeclareLaunchArgument('mode',      default_value='sign'),           # quantized|sign|passthrough
        DeclareLaunchArgument('turn_lin',  default_value='0.0'),            # 회전 중 전진(원하면 0.1 등)

        # ── 1) 매핑: /cmd_vel_raw -> /cmd_motion
        Node(
            package='coco', executable='motion_command_node', name='motion_mapper', output='screen',
            parameters=[{
                'in_cmd_topic':  '/cmd_vel_raw',
                'out_cmd_topic': '/cmd_motion',
                'mode': mode,
                'max_forward_lin': 0.6,
                'max_backward_lin': -0.3,
                'turn_ang': 1.0,
                'turn_lin': turn_lin,   # ← 회전 중 전진 속도
                'lin_step': 0.5,
                'ang_step': 1.0,
                'db_ratio': 0.2,
            }],
            arguments=['--ros-args', '--log-level', 'motion_mapper:=info']
        ),

        # ── 2) IMU 보정: /cmd_motion -> /cmd_vel
        Node(
            package='coco', executable='imu_stabilizer', name='imu_stab', output='screen',
            parameters=[{
                'in_cmd_topic':  '/cmd_motion',
                'out_cmd_topic': '/cmd_vel',
                'imu_topic': imu_topic,
                'imu_qos': imu_qos,               # best_effort | reliable
                'keep_ground_speed': True,
                'max_scale': 1.2,
                'max_lin_speed': 0.8,
                'roll_steer_gain': 0.15,
                'compensate_in_slow': True,
                'pitch_slow_deg': 10.0,
                'pitch_stop_deg': 50.0,
                'roll_slow_deg': 8.0,
                'roll_stop_deg': 20.0,
                'slow_scale': 0.3
            }],
            arguments=['--ros-args', '--log-level', 'imu_stabilizer:=info']
        ),
    ])
