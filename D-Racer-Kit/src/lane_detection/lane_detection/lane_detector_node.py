import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import cv2
import numpy as np

class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_node')
        
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.image_callback,
            image_qos)
            
        self.publisher_ = self.create_publisher(Float32, '/lane_info', 10)
        
        self.get_logger().info('OpenCV 기반 차선 인식 노드가 시작되었습니다!')

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if frame is None:
            self.get_logger().warning('이미지 디코딩 실패!')
            return

        height, width = frame.shape[:2]
        
        roi = frame[height//2:height, :]
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        edges = cv2.Canny(blur, 50, 150)
        
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=40, minLineLength=40, maxLineGap=20)
        
        left_lines = []
        right_lines = []
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 == x1: continue
                
                slope = (y2 - y1) / (x2 - x1)
                
                if slope < -0.3:
                    left_lines.append(line)
                elif slope > 0.3:
                    right_lines.append(line)
                    
        lane_center = width // 2
        
        if len(left_lines) > 0 and len(right_lines) > 0:
            left_x_avg = np.mean([l[0][0] for l in left_lines])
            right_x_avg = np.mean([l[0][0] for l in right_lines])
            lane_center = (left_x_avg + right_x_avg) / 2
            
        image_center = width / 2
        error = image_center - lane_center
        
        error_msg = Float32()
        error_msg.data = float(error)
        self.publisher_.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()