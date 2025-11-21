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

        # Name of the input tensor for onnx
        self.input_name = None

        # Resolution for normalization
        # Set as in MMAction2 pretrained model's val preprocessing
        # TODO: understand whether we need to change resolution to the same as the camera
        # 640
        self.img_height = 1080
        self.img_width = 1920

        # Change parameters without rebuilding pkg -> ros2 run f_gesture_recognition pose_classifier --ros-args -p model:=stgcn_ntu17_best.onnx -p num_frames:=40
        self.declare_parameter('model', 'stgcn_ntu60.onnx')
        self.declare_parameter('num_frames', 60)        # Num of frames for single input to the model

        # Read parameters
        model_name = self.get_parameter('model').get_parameter_value().string_value
        self.num_frames = self.get_parameter('num_frames').get_parameter_value().integer_value
        
        # Buffer storing last N frames of keypoints
        self.buffer = deque(maxlen=self.num_frames)

        # Path to model
        pkg_share = get_package_share_directory('f_gesture_recognition')
        model_path = os.path.join(pkg_share, 'models', model_name)

        self.get_logger().info(f"Model path: {model_path}")

        # Subscribe to keypoints
        self.sub_keypoints = self.create_subscription(PersonBodyArray, "/f_human_detection2/persons", self.keypoints_cb, 10)

        # Publisher for recognized actions/gestures
        self.pub_actions = self.create_publisher(PersonAction, "/f_gesture_recognition/actions", 10)

        # TODO: Interfaces

        # Load ONNX model session
        self.session = self.load_model(model_path)
        
        # Load action descriptions metadata (dictionary mapping class IDs → labels)
        self.action_descriptions = json.loads(self.session.get_modelmeta().custom_metadata_map["action_descriptions"])

        self.get_logger().info("Pose Classifier initialized")

    def load_model(self, model_path):
        """
        Load an ONNX model with CUDA if available, else CPU fallback.
        Logs model inputs and outputs for debugging.
        """

        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        try:
            session = ort.InferenceSession(model_path, providers=providers)

            self.get_logger().info(f'Loaded ONNX model: {model_path}')
            self.get_logger().info(f'Used provider: {session.get_providers()}')

            # Inspect and log model inputs names and types
            for i, input_info in enumerate(session.get_inputs()):
                self.get_logger().debug(f"Input {i}: name='{input_info.name}', shape={input_info.shape}, type={input_info.type}")

                # Save the first input's name for feeding inference
                if i == 0:
                    self.input_name = input_info.name

            # Inspect and log model outputs names and types
            for i, output_info in enumerate(session.get_outputs()):
                self.get_logger().debug(f"Output {i}: name='{output_info.name}', shape={output_info.shape}, type={output_info.type}")

            return session
        
        except Exception as e:
            self.get_logger().error(f'Failed to load ONNX model: {e}')
            return None

    def keypoints_cb(self, msg):
        """
        Callback executed when a new set of person keypoints is received.
        Preprocess input, run inference, postprocess output and publish results.
        """

        if self.session is None:
            self.get_logger().warning('ONNX session not available')
            return

        # Convert incoming keypoints into proper input shape
        inputs = self.preprocess_input(msg.persons)

        # Wait until buffer accumulates enough frames
        if inputs is None:
            self.get_logger().debug("Waiting for buffer to fill...")
            return

        # Run inference through ONNX session
        outputs = self.session.run(None, {self.input_name: inputs})

        # Decode model prediction
        label, confidence = self.postprocess_output(outputs)

        # Publish recognized action
        self.publish_detection(label, confidence)

    def preprocess_input(self, persons_msg):
        """
        Converts incoming PersonBody messages into normalized keypoint tensors.

        persons_msg: list[PersonBody]
        Returns tensor of shape: [num_batches, num_person, num_frames, num_joints, num_channels]
        """
        
        # Fill buffer with zero-frames until enough context exists
        # needed for first initialization
        while len(self.buffer) < self.num_frames:
            self.buffer.append(np.zeros((17, 3), dtype=np.float32))

        # TODO: Expand the buffer up to 200 frames and find a way to fill it

        # If no persons detected → use zeros for both persons
        if len(persons_msg) == 0:
            keypoints = np.zeros((17, 3), dtype=np.float32)
        else:
            # Currently only the first detected person is used
            # TODO: understand which person to choose or scan all persons
            p = persons_msg[0]

            # Keypoints array shape is [17, 3]
            keypoints = np.array(p.keypoints, dtype=np.float32).reshape(17, 3)

            # Normalize keypoints to [-1, 1] range relative to image center
            keypoints[:, 0] = (keypoints[:, 0] - self.img_width / 2) / (self.img_width / 2)
            keypoints[:, 1] = (keypoints[:, 1] - self.img_height / 2) / (self.img_height / 2)

        # Add new frame to buffer
        self.buffer.append(keypoints)

        # Not enough frames → delay inference
        if len(self.buffer) < self.num_frames:
            return None

        # Stack frames into one tensor: [T, 17, 3]
        frames = np.stack(list(self.buffer), axis=0)  # Shape: [T, 17, 3]

        # Model performs better with 2 persons, manually set second person as zeros
        person1_data = frames  # Shape: [T, 17, 3]
        person2_data = np.zeros_like(frames)  # Shape: [T, 17, 3]
        
        # Stack both persons: [2, T, 17, 3]
        both_persons = np.stack([person1_data, person2_data], axis=0)
        
        # Expand dims to match ST-GCN model input: [1, 2, T, 17, 3]
        inputs = both_persons[np.newaxis, ...]

        return inputs.astype(np.float32)
    
    def postprocess_output(self, outputs):
        """
        Interprets model outputs and extracts the most probable class label.
        """
        # Scores from model
        scores = outputs[0][0]  # Shape: (60,)
        
        # Apply softmax to convert scores to probabilities
        exp_scores = np.exp(scores - np.max(scores))  # subtract max for numerical stability
        probabilities = exp_scores / np.sum(exp_scores)
        
        # Determine predicted class index
        pred_class = np.argmax(probabilities)
        
        # Convert class index into human-readable label
        label = self.action_descriptions[f"A{int(pred_class)+1}"]
        
        # Extract model confidence score (now a proper probability in [0, 1])
        confidence = float(probabilities[pred_class])
        
        return label, confidence
    
    def publish_detection(self, label, confidence):
        """
        Publishes the recognized action as a PersonAction message.
        """
        action = PersonAction()
        action.header = Header()
        action.header.stamp = self.get_clock().now().to_msg()
        action.header.frame_id = ""
        action.label = label
        action.confidence = confidence

        self.pub_actions.publish(action)
        self.get_logger().info(f"Published action: {label} ({confidence:.2f})")

# Standart node lifecycle
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
