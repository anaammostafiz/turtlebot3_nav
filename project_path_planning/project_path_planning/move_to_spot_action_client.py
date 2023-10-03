import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class MoveToSpot(Node):
    def __init__(self):
        super().__init__("move_to_spot")
        self._action_client = ActionClient(self, NavigateToPose,'/navigate_to_pose')
        self.declare_parameter(name="spot_name")
        spot_name = self.get_parameter(name="spot_name").value
        #self.get_logger().info(str(spot_name))
        x_param = str(spot_name) + '.x'
        y_param = str(spot_name) + '.y'
        z_param = str(spot_name) + '.z'
        yaw_param = str(spot_name) + '.yaw'
        self.declare_parameters(
            namespace='',
            parameters=[
                (x_param, None),
                (y_param, None),
                (z_param, None),
                (yaw_param, None)
            ])
        x = self.get_parameter(x_param).value
        y = self.get_parameter(y_param).value
        z = self.get_parameter(z_param).value
        yaw = self.get_parameter(yaw_param).value

        self.send_goal(x,y,z,yaw)

    def send_goal(self, x, y, z, yaw):

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = yaw
        goal.pose = pose
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Received feedback: {0}, {1}, {2}'.format(feedback.current_pose.pose.position.x, feedback.current_pose.pose.position.y, feedback.current_pose.pose.orientation.w))
    
def main(args=None):
    rclpy.init(args=args)
    move_to_spot = MoveToSpot()
    rclpy.spin(move_to_spot)


if __name__ == '__main__':
    main()
