from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for the template demonstration.
    
    This launches three nodes:
    1. service_server_node - Provides the increment service
    2. action_server_node - Provides the increment action (calls the service)
    3. action_client_node - Sends a goal to the action server
    
    The nodes communicate as follows:
    - Action client sends a goal to action server
    - Action server calls service repeatedly to progress toward goal
    - Action server publishes feedback and then result
    """
    # ============================== STUDENT TODO ==============================
    # Update package/executable/node names after you rename your templates.
    # Add or remove nodes here to match your project architecture.
    # ========================================================================

    return LaunchDescription([
        Node(
            package='node_templates',
            executable='service_server_node',
            name='service_server_node',
            output='screen',
        ),
        Node(
            package='node_templates',
            executable='action_server_node',
            name='action_server_node',
            output='screen',
        ),
        Node(
            package='node_templates',
            executable='action_client_node',
            name='action_client_node',
            output='screen',
        ),
    ])
