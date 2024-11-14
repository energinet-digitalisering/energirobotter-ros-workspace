import argparse
from ultralytics import YOLO
import os
import glob
import shutil


def yolo_to_onnx(model_path, output_dir):
    # Load the YOLO model from the specified path
    model = YOLO(model_path)

    # Export the model to ONNX format
    print("Exporting model to ONNX format...")
    model.export(format="onnx")

    # Find the ONNX file in the output directory
    onnx_files = glob.glob(os.path.join(os.path.dirname(model_path), "*.onnx"))
    if not onnx_files:
        raise FileNotFoundError(
            "ONNX export failed. No ONNX file found in the model directory."
        )

    # Move the ONNX file to the specified output directory
    output_file = os.path.join(
        output_dir, os.path.basename(model_path).replace(".pt", ".onnx")
    )
    shutil.move(onnx_files[0], output_file)
    print(f"Model successfully exported to {output_file}")


if __name__ == "__main__":
    # Set up argument parsing
    parser = argparse.ArgumentParser(
        description="Convert a YOLOv8 model to ONNX format."
    )
    parser.add_argument(
        "model_path",
        type=str,
        help="Path to the YOLOv8 .pt model file (Ultralytics format)",
    )
    parser.add_argument(
        "output_dir", type=str, help="Directory to save the converted ONNX model file"
    )

    # Parse the arguments
    args = parser.parse_args()

    # Ensure output directory exists
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    # Run the conversion function
    yolo_to_onnx(args.model_path, args.output_dir)
