import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
import numpy as np
import cv2

from .backends import make_backend
from .postprocess import decode_yolo_pose
from .tracker import SimpleByteLike
from .viz.overlay import draw_overlay
from .viz.markers import make_markers
from .diagnostics import Profiler, build_diag
from f_human_detection_msgs.msg import PersonBody, PersonBodyArray
from geometry_msgs.msg import Point32
from .utils import resolve_model_path


class PoseNode(Node):
    def __init__(self):
        super().__init__("f_human_detection2")
        # Parameters (dev defaults to ONNX; prod overrides to TensorRT in prod.yaml)
        self.declare_parameters("", [
            ("backend", "onnxrt"),                 # "onnxrt" | "tensorrt"
            ("engine_path", "yolo11m-pose.plan"),  # TensorRT engine filename
            ("onnx_path", "yolo11m-pose.onnx"),    # ONNX filename
            ("precision", "fp16"),
            ("input_width", 640),
            ("input_height", 384),
            ("conf_thr", 0.25),
            ("iou_thr", 0.45),
            ("max_persons", 20),
            ("tracker_type", "byte"),
            ("publish_overlay", True),
            ("publish_markers", True),
            ("sync_mode", False),
            ("camera_frame", "camera_link"),
        ])

        self.bridge = CvBridge()
        self.prof = Profiler()

        # Read params
        iw = int(self.get_parameter("input_width").value)
        ih = int(self.get_parameter("input_height").value)
        self.backend_kind = str(self.get_parameter("backend").value)
        eng = resolve_model_path(str(self.get_parameter("engine_path").value))
        onnx = resolve_model_path(str(self.get_parameter("onnx_path").value))

        # Backend with automatic fallback to ONNX if TRT/PyCUDA unavailable
        try:
            self.backend = make_backend(self.backend_kind, eng, onnx, (iw, ih))
        except ImportError as e:
            self.get_logger().warn(f"{e}. Falling back to ONNXRuntime.")
            self.backend_kind = "onnxrt"
            self.backend = make_backend("onnxrt", eng, onnx, (iw, ih))

        # Tracker
        self.tracker = SimpleByteLike()

        # Pub/Sub
        self.sub_img = self.create_subscription(Image, "/image_raw", self.on_image, 10)
        self.pub_persons = self.create_publisher(PersonBodyArray, "/f_human_detection2/persons", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/f_human_detection2/metrics", 10)
        self.pub_overlay = self.create_publisher(Image, "/f_human_detection2/overlay", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/f_human_detection2/markers", 10)

        # Services
        self.srv_reload = self.create_service(Trigger, "/f_human_detection2/reload_engine", self.on_reload)

        self.sync_mode = bool(self.get_parameter("sync_mode").value)

    def on_reload(self, req, resp):
        """Reload backend with current parameters at runtime."""
        try:
            iw = int(self.get_parameter("input_width").value)
            ih = int(self.get_parameter("input_height").value)
            self.backend_kind = str(self.get_parameter("backend").value)
            eng = resolve_model_path(str(self.get_parameter("engine_path").value))
            onnx = resolve_model_path(str(self.get_parameter("onnx_path").value))
            try:
                self.backend = make_backend(self.backend_kind, eng, onnx, (iw, ih))
            except ImportError as e:
                self.get_logger().warn(f"{e}. Falling back to ONNXRuntime.")
                self.backend_kind = "onnxrt"
                self.backend = make_backend("onnxrt", eng, onnx, (iw, ih))
            resp.success = True
            resp.message = "reloaded"
        except Exception as e:
            resp.success = False
            resp.message = str(e)
        return resp

    def on_image(self, msg: Image):
        """Main callback: infer → postprocess → track → publish outputs."""
        _ = self.prof.tick()

        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        (boxes, scores, kpts), r, dwdh = self.backend.infer(cv_img)

        boxes, scores, kpts = decode_yolo_pose(
            (boxes, scores, kpts),
            conf_thr=float(self.get_parameter("conf_thr").value),
            iou_thr=float(self.get_parameter("iou_thr").value),
            max_persons=int(self.get_parameter("max_persons").value),
            orig_size=(cv_img.shape[1], cv_img.shape[0]),
            ratio=r, dwdh=dwdh
        )

        ids = self.tracker.update(boxes) if len(boxes) else []

        # Persons message
        arr = PersonBodyArray()
        arr.header = msg.header
        persons = []
        for i in range(len(boxes)):
            pb = PersonBody()
            pb.header = msg.header
            pb.id = int(ids[i])
            pb.score = float(scores[i])
            pb.keypoints = [Point32(x=float(kpts[i, j, 0]), y=float(kpts[i, j, 1]), z=0.0) for j in range(17)]
            pb.keypoint_scores = [float(kpts[i, j, 2]) for j in range(17)]
            pb.bbox = [float(x) for x in boxes[i].tolist()]  # xywh in pixels
            pb.source = f"{self.backend_kind}-yolo11m-pose"
            persons.append(pb)
        arr.persons = persons
        self.pub_persons.publish(arr)

        # Overlay image
        if bool(self.get_parameter("publish_overlay").value) and self.pub_overlay.get_subscription_count() > 0:
            overlay = draw_overlay(cv_img, boxes, kpts, ids)
            self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8"))

        # RViz markers
        if bool(self.get_parameter("publish_markers").value) and self.pub_markers.get_subscription_count() > 0:
            ma = make_markers(msg.header, kpts, ids)
            self.pub_markers.publish(MarkerArray(markers=ma.markers) if hasattr(ma, "markers") else ma)

        # Diagnostics
        dt = self.prof.tick()
        diag = build_diag("f_human_detection2", self.prof.fps, dt * 1000.0)
        self.pub_diag.publish(diag)

        if self.sync_mode:
            rclpy.spin_once(self, timeout_sec=0.0)


def main():
    rclpy.init()
    node = PoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
