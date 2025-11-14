import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from f_interfaces.msg import PersonBody, PersonBodyArray, PersonAction

import os
import onnxruntime as ort
import json
import numpy as np
from collections import deque

class PoseClassifier(Node):
    def __init__(self):
        super().__init__('pose_classifier')

        self.input_name = None

        self.img_height = 1080
        self.img_width = 1920

        self.declare_parameter('model', 'stgcn_ntu60_metadata.onnx')
        self.declare_parameter('num_frames', 10)
        # TODO: TensorRT params

        # Get model name (model.onnx by default)
        model_name = self.get_parameter('model').get_parameter_value().string_value

        self.num_frames = self.get_parameter('num_frames').get_parameter_value().integer_value
        self.buffer = deque(maxlen=self.num_frames)

        # Path to model
        pkg_share = get_package_share_directory('f_gesture_recognition')
        model_path = os.path.join(pkg_share, 'models', model_name)

        self.get_logger().info(f"Model path: {model_path}")

        # Subscribe to keypoints
        self.sub_keypoints = self.create_subscription(PersonBodyArray, "/f_human_detection2/persons", self.keypoints_cb, 10)

        # Publish detections
        self.pub_actions = self.create_publisher(PersonAction, "/f_gesture_recognition/actions", 10)

        # TODO: Interfaces

        # TODO: Check/chose .onnx or .engine model and handle appropriatly
        self.session = self.load_model(model_path)
        self.action_descriptions = json.loads(self.session.get_modelmeta().custom_metadata_map["action_descriptions"])

        self.get_logger().info("Pose Classifier initialized")

    def load_model(self, model_path):
        # Load onnx model
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        try:
            session = ort.InferenceSession(model_path, providers=providers)

            self.get_logger().info(f'Loaded ONNX model: {model_path}')

            for i, input_info in enumerate(session.get_inputs()):
                self.get_logger().debug(f"Input {i}: name='{input_info.name}', shape={input_info.shape}, type={input_info.type}")

                if i == 0:
                    self.input_name = input_info.name
            for i, output_info in enumerate(session.get_outputs()):
                self.get_logger().debug(f"Output {i}: name='{output_info.name}', shape={output_info.shape}, type={output_info.type}")

            return session
        
        except Exception as e:
            self.get_logger().error(f'Failed to load ONNX model: {e}')
            return None

    def keypoints_cb(self, msg):

        if self.session is None:
            self.get_logger().warning('ONNX session not available')
            return

        inputs = self.preprocess_input(msg.persons)

        if inputs is None:
            self.get_logger().debug("Waiting for buffer to fill...")
            return

        outputs = self.session.run(None, {self.input_name: inputs})

        label, confidence = self.postprocess_output(outputs)

        self.publish_detection(label, confidence)

    def preprocess_input(self, persons_msg):
        """
        persons_msg: list[PersonBody]
        Return shape: [1, 1, T, 17, 3]
        """
        
        if len(persons_msg) == 0:
            keypoints = np.zeros((17, 3), dtype=np.float32)
        else:
            p = persons_msg[0]      # Just take the first person for now
            # p.keypoints = [x1, y1, score1, ..., x17, y17, score17]

            # PreNormalize2D (from MMAction2) normalization
            keypoints = np.array(p.keypoints, dtype=np.float32).reshape(17, 2)
            keypoints[:, 0] = (keypoints[:, 0] - self.img_width / 2) / (self.img_width / 2)
            keypoints[:, 1] = (keypoints[:, 1] - self.img_height / 2) / (self.img_height / 2)

            keypoints = np.hstack([keypoints, np.ones((17,1), dtype=np.float32)])

        self.buffer.append(keypoints)

        if len(self.buffer) < self.num_frames:
            return None

        frames = np.stack(list(self.buffer), axis=0)     # [T, 17, 3]

        inputs = frames[np.newaxis, np.newaxis, ...]     # [1,1,T,17,3]

        return inputs.astype(np.float32)
    
    def postprocess_output(self, outputs):
        pred_class = np.argmax(outputs[0][0])

        label = self.action_descriptions[f"A{int(pred_class)+1}"]
        confidence = float(outputs[0][0][pred_class])

        return label, confidence
    
    def publish_detection(self, label, confidence):
        # Create message
        action = PersonAction()
        action.header = Header()
        action.header.stamp = self.get_clock().now().to_msg()
        action.header.frame_id = ""
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




