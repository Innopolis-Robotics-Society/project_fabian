import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from f_interfaces.msg import PersonBody, PersonBodyArray, PersonAction

import os
import onnxruntime as ort

from .process_utils import (
    preprocess_input,
    postprocess_output
)

class PoseClassifier(Node) :
    def __init__(self):
        super().__init__('pose_classifier')

        self.declare_parameter('model', 'model.onnx')
        # TODO: TensorRT params

        # Get model name (model.onnx by default)
        model_name = self.get_parameter('model').get_parameter_value().string_value

        # Path to model
        pkg_share = get_package_share_directory('f_gesture_recognition')
        model_path = os.path.join(pkg_share, 'models', model_name)

        self.get_logger().info(f"Model path: {model_path}")

        # Subscribe to keypoints
        self.sub_keypoints = self.create_subscription(PersonBodyArray, "/f_human_detection2/persons", self.keypoints_cb, 10)

        # Publish detections
        self.pub_actions = self.create_publisher(PersonAction, "/f_gesture_recognition/actions", 10)

        # TODO: Interfaces

        self.session = self.load_model(model_path)
        # TODO: Check/chose .onnx or .engine model and handle appropriatly

        self.get_logger().info("Pose Classifier initialized")

    def load_model(self, model_path):
        # Load onnx model
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        try:
            session = ort.InferenceSession(model_path, providers=providers)

            self.get_logger().info(f'Loaded ONNX model: {model_path}')

            for i, input_info in enumerate(self.session.get_inputs()):
                self.get_logger().debug(f"Input {i}: name='{input_info.name}', shape={input_info.shape}, type={input_info.type}")
            for i, output_info in enumerate(self.session.get_outputs()):
                self.get_logger().debug(f"Output {i}: name='{output_info.name}', shape={output_info.shape}, type={output_info.type}")
            
            return session
        except Exception as e:
            self.get_logger().error(f'Failed to load ONNX model: {e}')
            return None

    def keypoints_cb(self, msg):

        if self.session is None:
            self.get_logger().warning('ONNX session not available')
            return

        inputs = preprocess_input(msg.persons)

        outputs = self.session.run(None, inputs)

        # Get info from outputs dictionary
        label, confidence = postprocess_output(outputs)

        # Create message
        action = PersonAction()
        action.header = Header()
        action.header.stamp = self.get_clock().now().to_msg()
        action.header.frame_id = "base_link"    # Not sure
        action.label = label
        action.confidence = confidence

        self.pub_actions.publish(action)
        self.get_logger().info(f"Published action: {label} ({confidence:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = PoseClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  




