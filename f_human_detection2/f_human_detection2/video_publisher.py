import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__("video_publisher")
        self.declare_parameter("video_path", "")
        self.declare_parameter("fps", 30)

        path = self.get_parameter("video_path").value
        fps = float(self.get_parameter("fps").value)

        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open video: {path}")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "/image_raw", 10)
        self.timer = self.create_timer(1.0 / fps, self.step)

    def step(self):
        ret, frame = self.cap.read()
        if not ret:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VideoPublisher())
    rclpy.shutdown()
