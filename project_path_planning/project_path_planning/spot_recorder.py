import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml

class SpotRecorder(Node):

    def __init__(self):
        super().__init__('spot_recorder')
        self.subscriber = self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.listener_callback,10)

    def listener_callback(self, msg):
        try:
            with open('src/project_path_planning/config/spot-list_x.yaml', 'a') as file:
                yaml.dump(msg.pose.pose, file)
            self.get_logger().info('Successfully saved yaml')
        except:
            self.get_logger().error('Failed to save yaml')

def main(args=None):
    rclpy.init(args=args)
    spot_recorder = SpotRecorder()
    rclpy.spin(spot_recorder)
    spot_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main