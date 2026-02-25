#!/usr/bin/env python3
"""
Service Server Node Template
This node provides a simple increment service that adds 1 to a received number.

Students: replace this node/service naming and callback logic with your own interface and behavior.
"""

import rclpy
from rclpy.node import Node
from interface_templates.srv import IncrementNumber

# ============================== STUDENT TODO ==============================
# Replace `IncrementNumber` import with your own service interface.
# Update service and node names to match your project.
# ========================================================================


class ServiceServerNode(Node):
    def __init__(self):
        # Initialize this class as a ROS 2 node named 'service_server_node'.
        super().__init__('service_server_node')
        
        # Create the increment service server and bind the request callback.
        
        self.srv = self.create_service( # Instantiate the service server object
            IncrementNumber,            # Specify the service interface (defined in .srv file)
            'increment_number',         # Specify the name of the service (must match the client's service name)
            self.increment_callback     # Register the callback function that will execute when a service request is received
        )
        self.get_logger().info("Target service is available.")
    
    def increment_callback(self, request, response):
        """
        Service callback that receives a number and returns number + 1.
        """
        # ========================== STUDENT TODO ==========================
        # Replace this callback logic with your project-specific behavior.
        # Update request/response field names to match your `.srv` file.
        # =================================================================

        # Compute and return the incremented value.
        response.incremented_value = request.number + 1
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServiceServerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
