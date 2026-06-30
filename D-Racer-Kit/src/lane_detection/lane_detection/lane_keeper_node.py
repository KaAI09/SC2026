import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from control_msgs.msg import Control

class LaneKeeperNode(Node):
    def __init__(self):
        super().__init__('lane_keeper_node')
        
        self.subscription = self.create_subscription(
            Float32,
            '/lane_info',
            self.error_callback,
            10)
            
        self.publisher_ = self.create_publisher(Control, '/control', 10)
        
        self.Kp = 0.005
        self.throttle = 0.2
        
        self.get_logger().info('Lane Keeping 제어 노드가 시작되었습니다!')

    def error_callback(self, msg):
        error = msg.data
        
        steering = error * self.Kp
        
        steering = max(min(steering, 1.0), -1.0)
        
        control_msg = Control()
        control_msg.steering = float(steering)
        control_msg.throttle = float(self.throttle)
        
        self.publisher_.publish(control_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaneKeeperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()