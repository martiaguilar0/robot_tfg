from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_driver',
            node_executable='stm32_bridge_node',
            node_name='bridge',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='robot_driver',
            node_executable='inv_kinematics_node',
            node_name='kinematics',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='robot_driver',
            node_executable='odometry_node',
            node_name='odometry',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='robot_driver',
            node_executable='vision_localization_node',
            node_name='vision_localization',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='robot_driver',
            node_executable='ekf_node',
            node_name='mission',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='robot_driver',
            node_executable='navigation_node',
            node_name='navigation',
            output='screen',
        ),

        
    ])








        

