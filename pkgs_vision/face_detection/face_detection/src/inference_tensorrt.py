import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
from ultralytics import YOLO


class Box:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def size(self):
        return self.w * self.h


class InferenceFace:
    def __init__(
        self, model_path, trt_model_path, image_w, image_h, box_size_multiplier
    ):
        self.image_w = image_w
        self.image_h = image_h
        self.box_size_multiplier = box_size_multiplier

        # Initialize YOLO model
        self.model = YOLO(model_path)

        # Initialize TensorRT model
        self.trt_engine = self.load_engine(trt_model_path)
        self.context = self.trt_engine.create_execution_context()
        self.input_shape = self.context.get_binding_shape(0)
        self.output_shape = self.context.get_binding_shape(1)

        # Allocate buffers for input/output
        self.input_host = np.zeros(self.input_shape, dtype=np.float32)
        self.output_host = np.zeros(self.output_shape, dtype=np.float32)
        self.input_device = cuda.mem_alloc(self.input_host.nbytes)
        self.output_device = cuda.mem_alloc(self.output_host.nbytes)
        self.bindings = [int(self.input_device), int(self.output_device)]

    def box_closest(self, target, boxes):

        distance_min = 2 * self.image_w
        box = None
        for x, y, w, h in boxes:
            distance = np.linalg.norm([target[0] - x, target[1] - y])

            if distance < distance_min:
                distance_min = distance
                box = Box(x, y, w, h)

        return box

    def box_largest(self, boxes):

        largest_size = 0
        box = None
        for x, y, w, h in boxes:
            size = w * h

            if size > (largest_size):
                largest_size = size
                box = Box(x, y, w, h)

        return box

    def load_engine(self, trt_model_path):
        # Load TensorRT engine from file
        with open(trt_model_path, "rb") as f, trt.Runtime(
            trt.Logger(trt.Logger.WARNING)
        ) as runtime:
            return runtime.deserialize_cuda_engine(f.read())

    def preprocess_image(self, image):
        # Resize, normalize, and transpose image for TensorRT model (expected shape: 1x3xHxW)
        image_resized = cv2.resize(image, (self.input_shape[2], self.input_shape[1]))
        image_normalized = (image_resized / 255.0).astype(np.float32)
        return np.transpose(image_normalized, (2, 0, 1)).ravel()

    def parse_output(self, output):
        # Parse output to match YOLO’s result format (this depends on model output)
        # For simplicity, assume the output is a list of boxes in xywh format
        return [Box(x, y, w, h) for x, y, w, h in output.reshape(-1, 4)]

    def run_inference_yolo(self, image):
        # Run YOLO model inference
        return self.model.predict(source=image, show=False, verbose=False)

    def run_inference_trt(self, image):
        # Preprocess image for TensorRT
        input_data = self.preprocess_image(image)

        # Copy input to device, execute, and copy output back
        cuda.memcpy_htod(self.input_device, input_data)
        self.context.execute_v2(bindings=self.bindings)
        cuda.memcpy_dtoh(self.output_host, self.output_device)

        # Parse TensorRT output
        return self.parse_output(self.output_host)

    def detect_faces(self, image, use_trt=False):
        if use_trt:
            results = self.run_inference_trt(image)
        else:
            results = self.run_inference_yolo(image)

        # Convert results to list of boxes
        boxes = []
        for result in results:
            for box in result.boxes.xywh.tolist():
                boxes.append(box)

        if boxes:
            box_closest = self.box_closest((self.box_prev.x, self.box_prev.y), boxes)
            box_largest = self.box_largest(boxes)

            box_size_buffer = box_closest.size() * self.box_size_multiplier

            box_target = (
                box_largest
                if (box_largest.size() > (box_closest.size() + box_size_buffer))
                else box_closest
            )

            self.box_prev = box_target

        return results, box_target
