import numpy as np
from ultralytics import YOLO


class Box:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def size(self):
        return self.w * self.h


class InferenceYolo:
    def __init__(self, model_path, image_w, image_h, box_size_multiplier):

        self.image_w = image_w
        self.image_h = image_h
        self.box_size_multiplier = box_size_multiplier

        self.model = YOLO(model_path)
        self.box_prev = Box(image_w / 2, image_h / 2, 0, 0)

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

    def run_inference(self, image):

        # Run face detection model
        return self.model.predict(source=image, show=False, verbose=False)

    def detect_faces(self, image):

        results = self.run_inference(image)

        # Convert results to list of boxes
        boxes = []
        for result in results:
            for box in result.boxes.xywh.tolist():
                boxes.append(box)

        if len(boxes) != 0:  # Ensure detection

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
