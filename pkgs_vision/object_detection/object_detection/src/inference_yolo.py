import numpy as np
from ultralytics import YOLO


class Box:
    """A simple representation of a rectangular box in an image."""

    def __init__(self, x, y, w, h):
        """Initializes a new Box.

        Args:
            x (float): X-coordinate of the box center.
            y (float): Y-coordinate of the box center.
            w (float): Width of the box.
            h (float): Height of the box.
        """
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def size(self):
        """Returns the area of the box.

        Returns:
            float: Area of the box (width × height).
        """
        return self.w * self.h


class InferenceYolo:
    """YOLO-based object detection wrapper with heuristics for selecting target boxes."""

    def __init__(self, model_path, image_w, image_h, box_size_multiplier):
        """Initializes the InferenceYolo class.

        Args:
            model_path (str): Path to the YOLO model file.
            image_w (int): Width of the input images.
            image_h (int): Height of the input images.
            box_size_multiplier (float): Factor to buffer the previous box size when choosing targets.
        """
        self.image_w = image_w
        self.image_h = image_h
        self.box_size_multiplier = box_size_multiplier

        self.model = YOLO(model_path)
        self.box_prev = Box(image_w / 2, image_h / 2, 0, 0)

    def run_inference(self, image):
        """Runs inference on the input image.

        Args:
            image (np.ndarray or str): Image data or path to the image.

        Returns:
            List[ultralytics.engine.results.Results]: Inference results from the YOLO model.
        """
        return self.model.predict(source=image, show=False, verbose=False)

    def compute_target_box(self, results, class_of_interest):
        """Selects the target box based on inference results and heuristic rules.

        Prefers the box closest to the previous target unless a significantly
        larger box is detected.

        Args:
            results (List[ultralytics.engine.results.Results]): Results from `run_inference`.
            class_of_interest (int): Class ID to track (e.g., 0 for person).

        Returns:
            Box | None: Selected target box or None if no match found.
        """

        boxes = []
        for result in results:
            if result.boxes is None:
                continue

            xywh = result.boxes.xywh.cpu().numpy()
            cls = result.boxes.cls.cpu().numpy()

            for i in range(len(xywh)):
                if int(cls[i]) == class_of_interest:
                    x, y, w, h = xywh[i]
                    boxes.append((x, y, w, h))

        if not boxes:
            return None  # No box found for class_of_interest

        box_closest = self._box_closest((self.box_prev.x, self.box_prev.y), boxes)
        box_largest = self._box_largest(boxes)

        box_size_buffer = box_closest.size() * self.box_size_multiplier

        box_target = (
            box_largest
            if (box_largest.size() > (box_closest.size() + box_size_buffer))
            else box_closest
        )

        self.box_prev = box_target
        return box_target

    def _box_closest(self, target, boxes):
        """Finds the box closest to a given (x, y) target point.

        Args:
            target (Tuple[float, float]): Target (x, y) point.
            boxes (List[List[float]]): List of [x, y, w, h] boxes.

        Returns:
            Box: Closest box to the target point.
        """
        distance_min = 2 * self.image_w
        box = None
        for x, y, w, h in boxes:
            distance = np.linalg.norm([target[0] - x, target[1] - y])

            if distance < distance_min:
                distance_min = distance
                box = Box(x, y, w, h)

        return box

    def _box_largest(self, boxes):
        """Finds the largest box by area.

        Args:
            boxes (List[List[float]]): List of [x, y, w, h] boxes.

        Returns:
            Box: The largest box.
        """
        largest_size = 0
        box = None
        for x, y, w, h in boxes:
            size = w * h

            if size > (largest_size):
                largest_size = size
                box = Box(x, y, w, h)

        return box
