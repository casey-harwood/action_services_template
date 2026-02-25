#!/usr/bin/env python3
"""
Action Server Node Template
This action server receives a goal to increment to a target number.
It repeatedly calls the increment service until the target is reached.

Students: replace this node/action/service naming and logic with your own interfaces and behavior.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from interface_templates.srv import IncrementNumber
from interface_templates.action import IncrementToGoal

# ============================== STUDENT TODO ==============================
# Replace `IncrementNumber` and `IncrementToGoal` with your own interfaces.
# Update action/service names to match your project.
# ========================================================================

class ActionServerNode(Node):
    def __init__(self):
        # Initialize this class as a ROS 2 node named 'action_server_node'.
        super().__init__('action_server_node')
        
        # Create a SERVICE CLIENT used to call a service.
        # self.<service_client_name> = self.create_client(<Service Interface>, <service_name>)
        self.increment_client = self.create_client(IncrementNumber, 'increment_number')

        self.get_logger().info("Waiting for target service...")
        
        # Block execution until the service server has advertised the service we depend on. 
        while not self.increment_client.wait_for_service(timeout_sec=1.0):
            pass
        self.get_logger().info("Target service is available.")
        
        # Create the action server and register execute/goal/cancel callbacks.
        self._action_server = ActionServer(     # Instantiate the action server object
            self,                               # Define the scope of the server (this node)   
            IncrementToGoal,                    # Specify the action interface (defined in .action file)
            'increment_to_goal',                # Specify the name of the action (must match the client's action name)
            self.execute_callback,              # Register the callback function that will execute when a goal is accepted
            goal_callback=self.goal_callback,   # Register the callback function that will validate incoming goal requests
            cancel_callback=self.cancel_callback # Register the callback function that will handle cancellation requests
        )
        self.get_logger().info("Target action server is available.")
    
    def goal_callback(self, goal_request):
        """
        Validates incoming goal requests.
        Accepts any uint16 value as a valid goal.
        """
        # ========================== STUDENT TODO ==========================
        # Implement goal validation rules for your project.
        # Reject invalid goals based on your custom constraints.
        # =================================================================

        # Reject goals that request 1 or lower so every accepted goal performs work.
        if goal_request.target <= 1:
            return GoalResponse.REJECT
        
        # Accept any other valid goal request.
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Handle cancellation requests."""
        # ========================== STUDENT TODO ==========================
        # Add cleanup behavior required when a goal is canceled.
        # =================================================================

        # Accept cancel requests so active goals can stop cleanly.
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """
        Execute the action: increment from 1 to the target value.
        Calls the increment service repeatedly until target is reached.
        """
        # ========================== STUDENT TODO ==========================
        # Replace this execute logic with your own action workflow.
        # Update Goal/Feedback/Result field usage to match your `.action`.
        # =================================================================

        # Read the target value from the accepted goal request.
        target = goal_handle.request.target

        # Initialize the feedback message sent to the action client.
        feedback = IncrementToGoal.Feedback()
        feedback.status = "Starting increment process"
        feedback.current_value = 1

        goal_handle.publish_feedback(feedback)
        
        # Begin counting from 1 and increment via service calls.
        current_number = 1
        
        while current_number < target:
            # Exit early with a canceled result if the client requested cancel.
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = IncrementToGoal.Result()
                result.success = False
                result.final_value = current_number
                return result
            
            # Build the service request using the current number.
            request = IncrementNumber.Request()
            request.number = current_number
            
            # Call the increment service asynchronously and wait for response.
            future = self.increment_client.call_async(request)  # future is a placeholder for the result that will eventually be returned by the service call. It allows us to continue executing code while waiting for the service response.
            await future                                        # await is used to pause execution of this function until the future has a result (i.e., until the service call completes and returns a response).
            
            response = future.result()  # Once the future is complete, we can get the actual response from the service call using future.result()
            
            # Update progress using the incremented value from the service.
            current_number = response.incremented_value

            # Publish updated feedback so clients can track progress.
            feedback.current_value = current_number
            feedback.status = f"Current: {current_number}, Target: {target}"
            goal_handle.publish_feedback(feedback)

            # Stop looping once the target value is reached.
            if current_number == target:
                break
        
        # Mark the goal as succeeded and prepare the final result.
        goal_handle.succeed()
        
        result = IncrementToGoal.Result()
        result.success = True
        result.final_value = current_number

        return result


def main(args=None):
    rclpy.init(args=args)
    node = ActionServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
