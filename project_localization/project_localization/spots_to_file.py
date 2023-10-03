from custom_interfaces.srv import MyServiceMessage
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self._pose_sub = self.create_subscription(Odometry, 'odom', self.sub_callback, 10)
        self.srv = self.create_service(MyServiceMessage, 'my_srv_msg', self.my_srv_msg_callback)
        self._pose = Pose()

    def sub_callback(self,msg):
        self._pose = msg.pose.pose

    def my_srv_msg_callback(self, request, response):
        try:
            with open('spots_x.txt', 'a') as f:
                f.write(request.label)
                f.write('\n')
        except: 
            response.navigation_successfull = False
            response.message = 'Failed to write label'
        else:
            try: 
                with open('spots_x.txt', 'a') as f:
                    f.write(str(self._pose))
                    f.write('\n')
                response.navigation_successfull = True
                response.message = 'Successfully saved file'
            except:
                response.navigation_successfull = False
                response.message = 'Failed to write Pose'
            
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()