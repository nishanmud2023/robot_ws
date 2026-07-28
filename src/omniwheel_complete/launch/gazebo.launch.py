import os
import xacro
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('omniwheel_complete')
    xacro_file = os.path.join(pkg_share, 'urdf', 'omniwheel.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'urdf.rviz')

    # Process xacro to extract raw URDF string data
    robot_description_xml = xacro.process_file(xacro_file, mappings={}).toxml()

    # Automatically find the installation share directory containing your mesh assets
    # This guarantees Gazebo resolves package:// paths seamlessly inside Docker
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, '..')
    )

    return LaunchDescription([
        # Force environment path injection before processes start
        gazebo_resource_path,

        # 1. Gazebo Simulation Instantiation (Shapes environment)
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', os.path.join(pkg_share, 'worlds', 'hospital.sdf')],
            output='screen'
        ),

        # 2. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_xml,
                'use_sim_time': True
            }],
            output='screen'
        ),

        # 2b. EKF sensor fusion (odom + imu -> odometry/filtered)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[
                os.path.join(pkg_share, 'config', 'ekf.yaml'),
                {'use_sim_time': True}
            ],
            output='screen'
        ),

        # 3. Model Spawner
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'omniwheel_robot',
                '-string', robot_description_xml,
                '-x', '1.5',   # <-- Moved away from the red cube
                '-y', '1.5',   # <-- Moved away from the red cube
                '-z', '0.2'
            ],
            output='screen'
        ),

        # 4. Technical Parameter Bridge (Fully Corrected Sensor Pathing)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                
                # Handshaking the global /scan topic with explicit ROS frame string mapping
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
            ],
            parameters=[{
                # This forces the incoming laser packets to pair natively with your URDF link
                'lazy': False,
                'use_sim_time': True
            }],
            output='screen'
        ),

        # 5. RViz2 Display Environment (Now synchronized with simulation time)
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}], # <-- Crucial fix for link flickering
            output='screen'
        )
    ])
