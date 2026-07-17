import json
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool

try:
    import cv2
except ImportError:  # pragma: no cover - runtime dependency on the robot
    cv2 = None


@dataclass
class Detection:
    class_id: int
    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    def as_dict(self) -> Dict[str, Any]:
        width = max(0.0, self.x2 - self.x1)
        height = max(0.0, self.y2 - self.y1)
        return {
            "class_id": self.class_id,
            "class_name": self.class_name,
            "confidence": round(float(self.confidence), 4),
            "bbox_xyxy": [
                round(float(self.x1), 2),
                round(float(self.y1), 2),
                round(float(self.x2), 2),
                round(float(self.y2), 2),
            ],
            "bbox_xywh": [
                round(float(self.x1), 2),
                round(float(self.y1), 2),
                round(float(width), 2),
                round(float(height), 2),
            ],
        }


class M20ProYolov8Inspection(Node):
    def __init__(self) -> None:
        super().__init__("m20pro_yolov8_inspection")
        self._declare_parameters()

        if cv2 is None:
            raise RuntimeError("python3-opencv is required by m20pro_inspection")

        self.source_type = str(self.get_parameter("source_type").value)
        self.rtsp_url = str(self.get_parameter("rtsp_url").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_name = str(self.get_parameter("camera_name").value)
        self.backend_name = str(self.get_parameter("backend").value)
        self.model_path = self._resolve_model_path(str(self.get_parameter("model_path").value))
        self.input_size = int(self.get_parameter("input_size").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.max_detections = int(self.get_parameter("max_detections").value)
        self.publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self.publish_empty = bool(self.get_parameter("publish_empty_detections").value)
        self.publish_annotated = bool(self.get_parameter("publish_annotated_image").value)
        self.output_has_objectness = bool(self.get_parameter("output_has_objectness").value)
        self.reconnect_interval_s = float(self.get_parameter("reconnect_interval_s").value)
        self.event_conf_threshold = float(self.get_parameter("event_conf_threshold").value)
        self.event_min_interval_s = float(self.get_parameter("event_min_interval_s").value)
        self.event_classes = self._string_set(self.get_parameter("event_classes").value)
        self.enabled = bool(self.get_parameter("enabled").value)

        self.class_names = self._load_class_names()
        self.active_backend = "disabled"
        self.rknn = None
        self.onnx_session = None
        self.onnx_input_name = ""
        self.ultralytics_model = None
        self.ultralytics_device = str(self.get_parameter("ultralytics_device").value).strip()
        self.runtime_lock = threading.RLock()

        self.cap = None
        self.capture_lock = threading.Lock()
        self.capture_stop = threading.Event()
        self.capture_thread: Optional[threading.Thread] = None
        self.latest_capture_frame: Optional[np.ndarray] = None
        self.last_reconnect_time = 0.0
        self.latest_image: Optional[Image] = None
        self.last_event_time = 0.0
        self.warned_decode_shape = False
        self.started_at = time.time()
        self.frame_count = 0
        self.inference_count = 0
        self.last_frame_time: Optional[float] = None
        self.last_detection_time: Optional[float] = None
        self.last_inference_ms: Optional[float] = None
        self.last_detection_count = 0
        self.last_error = ""

        self.detections_pub = self.create_publisher(
            String,
            str(self.get_parameter("detections_topic").value),
            10,
        )
        self.events_pub = self.create_publisher(
            String,
            str(self.get_parameter("event_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.annotated_pub = self.create_publisher(
            Image,
            str(self.get_parameter("annotated_image_topic").value),
            qos_profile_sensor_data,
        )
        self.control_service = self.create_service(
            SetBool,
            "~/set_enabled",
            self._on_set_enabled,
        )

        if self.source_type == "image_topic":
            self.create_subscription(Image, self.image_topic, self._on_image, qos_profile_sensor_data)
            self.get_logger().info("inspection input: image topic %s" % self.image_topic)
        else:
            self.get_logger().info("inspection input: RTSP %s" % self.rtsp_url)

        if self.enabled:
            ok, message = self._activate_inspection()
            if not ok:
                self.enabled = False
                self.last_error = message
        else:
            self.get_logger().info("YOLO inspection is dormant; waiting for set_enabled")

        self.create_timer(1.0 / self.publish_rate_hz, self._tick)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            "YOLOv8 inspection process ready: enabled=%s backend=%s model=%s camera=%s"
            % (self.enabled, self.active_backend, self.model_path or "<none>", self.camera_name)
        )

    def destroy_node(self) -> bool:
        self._deactivate_inspection()
        return super().destroy_node()

    def _declare_parameters(self) -> None:
        self.declare_parameter("source_type", "rtsp")
        self.declare_parameter("rtsp_url", "rtsp://10.21.31.103:8554/video1")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_name", "front_wide")
        self.declare_parameter("enabled", False)
        self.declare_parameter("backend", "auto")
        self.declare_parameter("model_path", "")
        self.declare_parameter("class_names_path", "")
        self.declare_parameter("class_names", [""])
        self.declare_parameter("input_size", 640)
        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("max_detections", 100)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("reconnect_interval_s", 2.0)
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("publish_empty_detections", True)
        self.declare_parameter("detections_topic", "~/detections")
        self.declare_parameter("annotated_image_topic", "~/annotated_image")
        self.declare_parameter("event_topic", "~/events")
        self.declare_parameter("status_topic", "~/status")
        self.declare_parameter("event_classes", [""])
        self.declare_parameter("event_conf_threshold", 0.60)
        self.declare_parameter("event_min_interval_s", 2.0)
        self.declare_parameter("output_has_objectness", False)
        self.declare_parameter("ultralytics_device", "cpu")
        self.declare_parameter("process_nice_level", 10)

    def _on_set_enabled(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        requested = bool(request.data)
        with self.runtime_lock:
            if requested == self.enabled:
                response.success = True
                response.message = "YOLO already %s" % ("enabled" if requested else "disabled")
                self._publish_status()
                return response
            if requested:
                ok, message = self._activate_inspection()
                response.success = bool(ok)
                response.message = message
            else:
                self._deactivate_inspection()
                response.success = True
                response.message = "YOLO disabled and runtime resources released"
            self._publish_status()
            return response

    def _activate_inspection(self) -> Tuple[bool, str]:
        with self.runtime_lock:
            if self.enabled and self.active_backend not in ("disabled", ""):
                return True, "YOLO already enabled"
            self.last_error = ""
            self.active_backend = "disabled"
            try:
                self._load_backend()
                if self.active_backend == "dry_run":
                    raise RuntimeError("YOLO model backend is unavailable; refusing dry-run activation")
                self._apply_process_nice_level()
                self.enabled = True
                self._start_capture()
            except Exception as exc:
                self.enabled = False
                self.last_error = str(exc) or exc.__class__.__name__
                self._release_backend()
                self.get_logger().error("failed to enable YOLO inspection: %s" % self.last_error)
                return False, self.last_error
            self.get_logger().info("YOLO inspection enabled: backend=%s" % self.active_backend)
            return True, "YOLO enabled with %s backend" % self.active_backend

    def _deactivate_inspection(self) -> None:
        with self.runtime_lock:
            self.enabled = False
            self._stop_capture()
            self._release_backend()
            self.latest_image = None
            self.last_frame_time = None
            self.last_detection_time = None
            self.last_inference_ms = None
            self.last_detection_count = 0
            self.last_error = ""
            self.get_logger().info("YOLO inspection disabled; runtime resources released")

    def _start_capture(self) -> None:
        if self.source_type == "image_topic" or self.capture_thread is not None:
            return
        self.capture_stop = threading.Event()
        self.capture_thread = threading.Thread(
            target=self._capture_loop,
            name="m20pro_inspection_capture",
            daemon=True,
        )
        self.capture_thread.start()

    def _stop_capture(self) -> None:
        self.capture_stop.set()
        thread = self.capture_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        # Normal shutdown lets the capture thread release its own VideoCapture,
        # avoiding a concurrent release while OpenCV is inside read(). Only use
        # the fallback release after the bounded join if a backend is stuck.
        if thread is not None and thread.is_alive():
            with self.capture_lock:
                cap = self.cap
                self.cap = None
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:
                        pass
            thread.join(timeout=1.0)
        self.capture_thread = None
        with self.capture_lock:
            self.latest_capture_frame = None

    def _release_backend(self) -> None:
        if self.rknn is not None:
            try:
                self.rknn.release()
            except Exception:
                pass
        self.rknn = None
        self.onnx_session = None
        self.onnx_input_name = ""
        self.ultralytics_model = None
        self.active_backend = "disabled"

    def _apply_process_nice_level(self) -> None:
        nice_level = max(0, min(19, int(self.get_parameter("process_nice_level").value)))
        try:
            os.setpriority(os.PRIO_PROCESS, 0, nice_level)
        except (AttributeError, OSError) as exc:
            self.get_logger().warning("failed to set inspection nice level %d: %s" % (nice_level, exc))

    def _load_class_names(self) -> List[str]:
        names = [str(item) for item in self.get_parameter("class_names").value if str(item)]
        if names:
            return names

        names_path = self._resolve_model_path(str(self.get_parameter("class_names_path").value))
        if names_path and os.path.exists(names_path):
            with open(names_path, "r", encoding="utf-8") as f:
                return [line.strip() for line in f if line.strip()]
        return []

    @staticmethod
    def _resolve_model_path(path: str) -> str:
        if not path or os.path.exists(path):
            return path

        basename = os.path.basename(path)
        candidates = [
            os.path.join(os.getcwd(), "src", "m20pro_inspection", "models", basename),
            os.path.join(os.path.expanduser("~"), "m20pro_models", basename),
        ]

        install_marker = os.sep + "install" + os.sep
        if install_marker in path:
            workspace_root = path.split(install_marker, maxsplit=1)[0]
            candidates.insert(
                0,
                os.path.join(workspace_root, "src", "m20pro_inspection", "models", basename),
            )

        for candidate in candidates:
            if os.path.exists(candidate):
                return candidate
        return path

    @staticmethod
    def _string_set(values: Sequence[Any]) -> set:
        return {str(value) for value in values if str(value)}

    def _load_backend(self) -> None:
        requested = self.backend_name.lower().strip()
        auto_selected = requested == "auto"
        model_path = self.model_path

        if auto_selected:
            ext = os.path.splitext(model_path)[1].lower()
            if ext == ".rknn":
                requested = "rknn"
            elif ext == ".onnx":
                requested = "onnx"
            elif ext in (".pt", ".pth"):
                requested = "ultralytics"
            else:
                requested = "dry_run"
        elif requested in ("pt", "torch", "pytorch"):
            requested = "ultralytics"

        if requested != "dry_run" and (not model_path or not os.path.exists(model_path)):
            self.get_logger().warning(
                "inspection model not found: %s; node will publish empty dry-run results"
                % (model_path or "<empty>")
            )
            requested = "dry_run"

        try:
            if requested == "rknn":
                self._load_rknn(model_path)
            elif requested == "onnx":
                self._load_onnx(model_path)
            elif requested == "ultralytics":
                self._load_ultralytics(model_path)
            elif requested == "dry_run":
                self.active_backend = "dry_run"
            else:
                raise RuntimeError("unsupported inspection backend: %s" % requested)
        except RuntimeError as exc:
            if not auto_selected:
                raise
            self.get_logger().warning(
                "inspection backend %s unavailable in auto mode: %s; "
                "node will publish empty dry-run results" % (requested, exc)
            )
            self.active_backend = "dry_run"

    def _load_rknn(self, model_path: str) -> None:
        try:
            from rknnlite.api import RKNNLite
        except ImportError as exc:
            raise RuntimeError("rknnlite is required for RK3588 RKNN inference") from exc

        rknn = RKNNLite()
        ret = rknn.load_rknn(model_path)
        if ret != 0:
            raise RuntimeError("failed to load RKNN model: %s" % model_path)

        core_mask = getattr(RKNNLite, "NPU_CORE_AUTO", None)
        if core_mask is None:
            ret = rknn.init_runtime()
        else:
            ret = rknn.init_runtime(core_mask=core_mask)
        if ret != 0:
            raise RuntimeError("failed to init RKNN runtime")

        self.rknn = rknn
        self.active_backend = "rknn"

    def _load_onnx(self, model_path: str) -> None:
        try:
            import onnxruntime as ort
        except ImportError as exc:
            raise RuntimeError("onnxruntime is required for ONNX inspection inference") from exc

        self.onnx_session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
        self.onnx_input_name = self.onnx_session.get_inputs()[0].name
        self.active_backend = "onnx"

    def _load_ultralytics(self, model_path: str) -> None:
        self._install_numpy_pickle_compat()
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is required for .pt inspection inference; "
                "install torch and ultralytics, or convert the model to RKNN/ONNX"
            ) from exc

        self.ultralytics_model = YOLO(model_path)
        self.active_backend = "ultralytics"

    @staticmethod
    def _install_numpy_pickle_compat() -> None:
        try:
            import numpy as _np
        except Exception:
            return
        sys.modules.setdefault("numpy._core", _np.core)
        for name in (
            "multiarray",
            "umath",
            "_multiarray_umath",
            "numeric",
            "fromnumeric",
            "shape_base",
        ):
            try:
                sys.modules.setdefault(
                    "numpy._core." + name,
                    __import__("numpy.core." + name, fromlist=["*"]),
                )
            except Exception:
                pass

    def _on_image(self, msg: Image) -> None:
        if self.enabled:
            self.latest_image = msg

    def _tick(self) -> None:
        # RKNNLite and OpenCV resources must never be released while inference
        # or a capture transition is using them. The service callback and timer
        # may run on different executor threads on the real robot.
        if not self.runtime_lock.acquire(blocking=False):
            return
        try:
            self._tick_locked()
        finally:
            self.runtime_lock.release()

    def _tick_locked(self) -> None:
        if not self.enabled:
            return
        frame, stamp = self._read_frame()
        if frame is None:
            self._publish_status()
            return

        self.frame_count += 1
        self.last_frame_time = time.time()
        detections: List[Detection] = []
        if self.active_backend == "ultralytics":
            try:
                start = time.monotonic()
                detections = self._infer_ultralytics(frame)
                self.last_inference_ms = (time.monotonic() - start) * 1000.0
                self.inference_count += 1
                self.last_error = ""
            except Exception as exc:
                self.last_error = "inspection inference failed: %s" % exc
                self.get_logger().warning(self.last_error)
                self._publish_status()
                return
        elif self.active_backend != "dry_run":
            input_tensor, meta = self._preprocess(frame)
            try:
                start = time.monotonic()
                outputs = self._infer(input_tensor)
                detections = self._decode_outputs(outputs, meta, frame.shape[:2])
                self.last_inference_ms = (time.monotonic() - start) * 1000.0
                self.inference_count += 1
                self.last_error = ""
            except Exception as exc:
                self.last_error = "inspection inference failed: %s" % exc
                self.get_logger().warning(self.last_error)
                self._publish_status()
                return

        self.last_detection_count = len(detections)
        self.last_detection_time = time.time()
        if detections or self.publish_empty:
            self._publish_detections(detections, frame.shape, stamp)
        if self.publish_annotated:
            annotated = self._draw_detections(frame.copy(), detections)
            self.annotated_pub.publish(self._bgr_to_msg(annotated, stamp))
        self._publish_event_if_needed(detections, stamp)
        self._publish_status()

    def _read_frame(self) -> Tuple[Optional[np.ndarray], Any]:
        if self.source_type == "image_topic":
            if self.latest_image is None:
                return None, self.get_clock().now().to_msg()
            msg = self.latest_image
            self.latest_image = None
            return self._image_msg_to_bgr(msg), msg.header.stamp

        with self.capture_lock:
            frame = self.latest_capture_frame
            self.latest_capture_frame = None
        if frame is None:
            return None, self.get_clock().now().to_msg()
        return frame, self.get_clock().now().to_msg()

    def _capture_loop(self) -> None:
        os.environ.setdefault(
            "OPENCV_FFMPEG_CAPTURE_OPTIONS",
            "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay",
        )
        current_cap = None
        while not self.capture_stop.is_set():
            with self.capture_lock:
                if self.capture_stop.is_set():
                    break
                current_cap = self.cap
                needs_open = current_cap is None or not current_cap.isOpened()
            if needs_open:
                self.last_reconnect_time = time.monotonic()
                current_cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
                current_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if not current_cap.isOpened():
                    current_cap.release()
                    self.last_error = "failed to open RTSP stream: %s" % self.rtsp_url
                    self.capture_stop.wait(max(0.2, self.reconnect_interval_s))
                    continue
                with self.capture_lock:
                    self.cap = current_cap

            # Keep release and read mutually exclusive. OpenCV backends are
            # not safe when VideoCapture.release() races VideoCapture.read().
            with self.capture_lock:
                if self.capture_stop.is_set():
                    break
                ok, frame = current_cap.read()
            if not ok or frame is None:
                if self.capture_stop.is_set():
                    break
                self.last_error = "RTSP frame read failed; reconnecting"
                with self.capture_lock:
                    current_cap.release()
                    if self.cap is current_cap:
                        self.cap = None
                self.capture_stop.wait(max(0.2, self.reconnect_interval_s))
                continue
            with self.capture_lock:
                self.latest_capture_frame = frame
            self.last_error = ""
        if current_cap is not None:
            with self.capture_lock:
                if self.cap is current_cap:
                    self.cap = None
                try:
                    current_cap.release()
                except Exception:
                    pass

    def _image_msg_to_bgr(self, msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        if encoding in ("bgr8", "rgb8"):
            image = self._reshape_image(msg, 3)
            if encoding == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            return image
        if encoding in ("bgra8", "rgba8"):
            image = self._reshape_image(msg, 4)
            code = cv2.COLOR_BGRA2BGR if encoding == "bgra8" else cv2.COLOR_RGBA2BGR
            return cv2.cvtColor(image, code)
        if encoding in ("mono8", "8uc1"):
            image = self._reshape_image(msg, 1)
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        raise RuntimeError("unsupported image encoding: %s" % msg.encoding)

    @staticmethod
    def _reshape_image(msg: Image, channels: int) -> np.ndarray:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        rows = data.reshape((msg.height, msg.step))
        image = rows[:, : msg.width * channels].reshape((msg.height, msg.width, channels))
        return image.copy()

    def _preprocess(self, frame_bgr: np.ndarray) -> Tuple[np.ndarray, Dict[str, float]]:
        image_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        h, w = image_rgb.shape[:2]
        scale = min(self.input_size / float(w), self.input_size / float(h))
        new_w = int(round(w * scale))
        new_h = int(round(h * scale))
        resized = cv2.resize(image_rgb, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        pad_x = (self.input_size - new_w) / 2.0
        pad_y = (self.input_size - new_h) / 2.0
        left = int(round(pad_x - 0.1))
        right = int(round(pad_x + 0.1))
        top = int(round(pad_y - 0.1))
        bottom = int(round(pad_y + 0.1))
        padded = cv2.copyMakeBorder(
            resized,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )

        if self.active_backend == "onnx":
            tensor = padded.astype(np.float32) / 255.0
            tensor = np.transpose(tensor, (2, 0, 1))[np.newaxis, ...]
        else:
            tensor = padded[np.newaxis, ...]

        return tensor, {
            "scale": scale,
            "pad_x": float(left),
            "pad_y": float(top),
            "input_w": float(self.input_size),
            "input_h": float(self.input_size),
        }

    def _infer(self, input_tensor: np.ndarray) -> List[np.ndarray]:
        if self.active_backend == "rknn":
            outputs = self.rknn.inference(inputs=[input_tensor])
            return [np.asarray(output) for output in outputs]
        if self.active_backend == "onnx":
            outputs = self.onnx_session.run(None, {self.onnx_input_name: input_tensor})
            return [np.asarray(output) for output in outputs]
        return []

    @staticmethod
    def _tensor_to_numpy(value: Any) -> np.ndarray:
        if hasattr(value, "detach"):
            value = value.detach()
        if hasattr(value, "cpu"):
            value = value.cpu()
        return np.asarray(value)

    def _infer_ultralytics(self, frame_bgr: np.ndarray) -> List[Detection]:
        if self.ultralytics_model is None:
            return []

        kwargs: Dict[str, Any] = {
            "source": frame_bgr,
            "imgsz": self.input_size,
            "conf": self.conf_threshold,
            "iou": self.iou_threshold,
            "max_det": self.max_detections,
            "verbose": False,
        }
        if self.ultralytics_device:
            kwargs["device"] = self.ultralytics_device

        results = self.ultralytics_model.predict(**kwargs)
        if not results:
            return []

        boxes = getattr(results[0], "boxes", None)
        if boxes is None or len(boxes) == 0:
            return []

        xyxy = self._tensor_to_numpy(boxes.xyxy)
        conf = self._tensor_to_numpy(boxes.conf)
        cls = self._tensor_to_numpy(boxes.cls).astype(np.int32)

        detections: List[Detection] = []
        for idx in range(min(len(xyxy), self.max_detections)):
            class_id = int(cls[idx])
            x1, y1, x2, y2 = [float(value) for value in xyxy[idx].tolist()]
            detections.append(
                Detection(
                    class_id=class_id,
                    class_name=self._class_name(class_id, self._ultralytics_name(class_id)),
                    confidence=float(conf[idx]),
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                )
            )
        detections.sort(key=lambda item: item.confidence, reverse=True)
        return detections

    def _decode_outputs(
        self,
        outputs: Iterable[np.ndarray],
        meta: Dict[str, float],
        image_shape: Tuple[int, int],
    ) -> List[Detection]:
        boxes: List[List[float]] = []
        scores: List[float] = []
        class_ids: List[int] = []

        for output in outputs:
            for matrix in self._prediction_matrices(output):
                if matrix.shape[1] == 6:
                    self._collect_nms_rows(matrix, meta, image_shape, boxes, scores, class_ids)
                elif matrix.shape[1] >= 5:
                    self._collect_yolov8_rows(matrix, meta, image_shape, boxes, scores, class_ids)

        if not boxes:
            return []

        keep = self._class_aware_nms(np.asarray(boxes), np.asarray(scores), np.asarray(class_ids))
        detections: List[Detection] = []
        for idx in keep[: self.max_detections]:
            class_id = int(class_ids[idx])
            detections.append(
                Detection(
                    class_id=class_id,
                    class_name=self._class_name(class_id),
                    confidence=float(scores[idx]),
                    x1=float(boxes[idx][0]),
                    y1=float(boxes[idx][1]),
                    x2=float(boxes[idx][2]),
                    y2=float(boxes[idx][3]),
                )
            )
        detections.sort(key=lambda item: item.confidence, reverse=True)
        return detections

    def _prediction_matrices(self, output: np.ndarray) -> Iterable[np.ndarray]:
        arr = np.asarray(output)
        arr = np.squeeze(arr)
        if arr.ndim == 2:
            yield self._normalize_prediction_matrix(arr)
            return
        if arr.ndim == 3 and arr.shape[0] == 1:
            yield self._normalize_prediction_matrix(np.squeeze(arr, axis=0))
            return
        if not self.warned_decode_shape:
            self.get_logger().warning(
                "unsupported YOLO output shape %s; export a single-output YOLOv8 ONNX/RKNN if decoding is empty"
                % (tuple(output.shape),)
            )
            self.warned_decode_shape = True

    @staticmethod
    def _normalize_prediction_matrix(arr: np.ndarray) -> np.ndarray:
        if arr.shape[0] < arr.shape[1] and arr.shape[0] <= 512:
            arr = arr.T
        return arr.astype(np.float32, copy=False)

    def _collect_nms_rows(
        self,
        matrix: np.ndarray,
        meta: Dict[str, float],
        image_shape: Tuple[int, int],
        boxes: List[List[float]],
        scores: List[float],
        class_ids: List[int],
    ) -> None:
        for row in matrix:
            score = float(row[4])
            if score < self.conf_threshold:
                continue
            class_id = int(row[5])
            boxes.append(self._scale_box(row[:4], meta, image_shape, already_xyxy=True))
            scores.append(score)
            class_ids.append(class_id)

    def _collect_yolov8_rows(
        self,
        matrix: np.ndarray,
        meta: Dict[str, float],
        image_shape: Tuple[int, int],
        boxes: List[List[float]],
        scores: List[float],
        class_ids: List[int],
    ) -> None:
        raw_boxes = matrix[:, :4]
        raw_scores = matrix[:, 4:]
        if self.output_has_objectness:
            objectness = matrix[:, 4:5]
            raw_scores = matrix[:, 5:] * objectness

        if raw_scores.size == 0:
            return
        if np.nanmax(raw_scores) > 1.0 or np.nanmin(raw_scores) < 0.0:
            raw_scores = 1.0 / (1.0 + np.exp(-raw_scores))

        best_class = np.argmax(raw_scores, axis=1)
        best_score = raw_scores[np.arange(raw_scores.shape[0]), best_class]
        selected = np.where(best_score >= self.conf_threshold)[0]
        for idx in selected:
            boxes.append(self._scale_box(raw_boxes[idx], meta, image_shape, already_xyxy=False))
            scores.append(float(best_score[idx]))
            class_ids.append(int(best_class[idx]))

    def _scale_box(
        self,
        box: Sequence[float],
        meta: Dict[str, float],
        image_shape: Tuple[int, int],
        already_xyxy: bool,
    ) -> List[float]:
        values = np.asarray(box, dtype=np.float32).copy()
        if np.nanmax(values) <= 2.0:
            values[[0, 2]] *= meta["input_w"]
            values[[1, 3]] *= meta["input_h"]

        if already_xyxy:
            x1, y1, x2, y2 = values.tolist()
        else:
            cx, cy, w, h = values.tolist()
            x1 = cx - w / 2.0
            y1 = cy - h / 2.0
            x2 = cx + w / 2.0
            y2 = cy + h / 2.0

        x1 = (x1 - meta["pad_x"]) / meta["scale"]
        y1 = (y1 - meta["pad_y"]) / meta["scale"]
        x2 = (x2 - meta["pad_x"]) / meta["scale"]
        y2 = (y2 - meta["pad_y"]) / meta["scale"]

        height, width = image_shape
        return [
            max(0.0, min(float(width - 1), x1)),
            max(0.0, min(float(height - 1), y1)),
            max(0.0, min(float(width - 1), x2)),
            max(0.0, min(float(height - 1), y2)),
        ]

    def _class_aware_nms(
        self,
        boxes: np.ndarray,
        scores: np.ndarray,
        class_ids: np.ndarray,
    ) -> List[int]:
        keep: List[int] = []
        for class_id in np.unique(class_ids):
            indices = np.where(class_ids == class_id)[0]
            class_keep = self._nms_indices(boxes[indices], scores[indices])
            keep.extend(indices[class_keep].tolist())
        keep.sort(key=lambda idx: float(scores[idx]), reverse=True)
        return keep

    def _nms_indices(self, boxes: np.ndarray, scores: np.ndarray) -> List[int]:
        if boxes.size == 0:
            return []
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
        order = scores.argsort()[::-1]
        keep: List[int] = []

        while order.size > 0:
            i = int(order[0])
            keep.append(i)
            if order.size == 1:
                break
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
            union = areas[i] + areas[order[1:]] - inter
            iou = inter / np.maximum(union, 1e-6)
            order = order[1:][iou <= self.iou_threshold]
        return keep

    def _ultralytics_name(self, class_id: int) -> str:
        names = getattr(self.ultralytics_model, "names", None)
        if isinstance(names, dict):
            value = names.get(class_id, names.get(str(class_id), ""))
            return str(value) if value else ""
        if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
            return str(names[class_id])
        return ""

    def _class_name(self, class_id: int, fallback: str = "") -> str:
        if 0 <= class_id < len(self.class_names):
            return self.class_names[class_id]
        if fallback:
            return fallback
        return "class_%d" % class_id

    def _publish_detections(self, detections: List[Detection], frame_shape: Sequence[int], stamp: Any) -> None:
        height, width = int(frame_shape[0]), int(frame_shape[1])
        payload = {
            "stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
            "camera": self.camera_name,
            "source_type": self.source_type,
            "backend": self.active_backend,
            "model_path": self.model_path,
            "image_width": width,
            "image_height": height,
            "count": len(detections),
            "detections": [det.as_dict() for det in detections],
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.detections_pub.publish(msg)

    def _publish_status(self) -> None:
        now = time.time()
        frame_age = round(now - self.last_frame_time, 3) if self.last_frame_time is not None else None
        backend_ready = self.enabled and self.active_backend not in ("disabled", "dry_run", "")
        ready = (
            backend_ready
            and frame_age is not None
            and frame_age <= max(2.0, self.reconnect_interval_s * 2.0)
            and not self.last_error
        )
        payload = {
            "stamp": now,
            "uptime_s": round(now - self.started_at, 3),
            "enabled": self.enabled,
            "state": "ready" if ready else ("error" if self.last_error else ("starting" if self.enabled else "disabled")),
            "camera": self.camera_name,
            "source_type": self.source_type,
            "requested_backend": self.backend_name,
            "backend": self.active_backend,
            "model_path": self.model_path if self.enabled else None,
            "model_loaded": self.enabled and self.active_backend not in ("disabled", "dry_run", ""),
            "ready": ready,
            "publish_rate_hz": self.publish_rate_hz,
            "frame_count": self.frame_count,
            "inference_count": self.inference_count,
            "last_frame_age_s": frame_age,
            "last_detection_age_s": (
                round(now - self.last_detection_time, 3) if self.last_detection_time is not None else None
            ),
            "last_inference_ms": (
                round(self.last_inference_ms, 2) if self.last_inference_ms is not None else None
            ),
            "last_detection_count": self.last_detection_count,
            "last_error": self.last_error or None,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.status_pub.publish(msg)

    def _publish_event_if_needed(self, detections: List[Detection], stamp: Any) -> None:
        candidates = [
            det
            for det in detections
            if det.confidence >= self.event_conf_threshold and self._event_class_allowed(det)
        ]
        if not candidates:
            return
        now = time.monotonic()
        if now - self.last_event_time < self.event_min_interval_s:
            return
        self.last_event_time = now

        payload = {
            "stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
            "camera": self.camera_name,
            "type": "inspection_detection",
            "count": len(candidates),
            "top_detection": candidates[0].as_dict(),
            "detections": [det.as_dict() for det in candidates],
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.events_pub.publish(msg)

    def _event_class_allowed(self, det: Detection) -> bool:
        if not self.event_classes:
            return True
        return det.class_name in self.event_classes or str(det.class_id) in self.event_classes

    def _draw_detections(self, image: np.ndarray, detections: List[Detection]) -> np.ndarray:
        for det in detections:
            x1, y1, x2, y2 = [int(round(value)) for value in (det.x1, det.y1, det.x2, det.y2)]
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 200, 255), 2)
            label = "%s %.2f" % (det.class_name, det.confidence)
            cv2.putText(
                image,
                label,
                (x1, max(18, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 200, 255),
                2,
                cv2.LINE_AA,
            )
        return image

    def _bgr_to_msg(self, image: np.ndarray, stamp: Any) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_name
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(image.shape[1] * 3)
        msg.data = image.tobytes()
        return msg


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = M20ProYolov8Inspection()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
