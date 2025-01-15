"""
Transform (rotation) calculator.
Takes and input matrix, angles in degrees to rotate, and an axis to rotate it around, and then spits out the rotated matrix.
"""

import numpy as np

# Set NumPy to print with a maximum of 5 decimal places and suppress scientific notation
np.set_printoptions(precision=5, suppress=True)


def rotate_matrix(matrix, angle_degrees, axis):
    """
    Rotates the given 4x4 matrix by the specified angle around the given axis.

    Parameters:
        matrix (numpy.ndarray): The input 4x4 matrix to rotate.
        angle_degrees (float): The angle of rotation in degrees.
        axis (str): The axis of rotation ('x', 'y', or 'z').

    Returns:
        numpy.ndarray: The rotated 4x4 matrix.
    """
    angle_radians = np.deg2rad(angle_degrees)

    # Define rotation matrices for each axis
    if axis == "x":
        rotation_matrix = np.array(
            [
                [1, 0, 0, 0],
                [0, np.cos(angle_radians), -np.sin(angle_radians), 0],
                [0, np.sin(angle_radians), np.cos(angle_radians), 0],
                [0, 0, 0, 1],
            ]
        )
    elif axis == "y":
        rotation_matrix = np.array(
            [
                [np.cos(angle_radians), 0, np.sin(angle_radians), 0],
                [0, 1, 0, 0],
                [-np.sin(angle_radians), 0, np.cos(angle_radians), 0],
                [0, 0, 0, 1],
            ]
        )
    elif axis == "z":
        rotation_matrix = np.array(
            [
                [np.cos(angle_radians), -np.sin(angle_radians), 0, 0],
                [np.sin(angle_radians), np.cos(angle_radians), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
    else:
        raise ValueError("Invalid axis. Choose 'x', 'y', or 'z'.")

    # Apply the rotation
    rotated_matrix = rotation_matrix @ matrix
    return rotated_matrix


if __name__ == "__main__":

    # Example 4x4 identity matrix
    identity_matrix = np.eye(4)

    matrix = np.array(
        [
            [0, 0, -1, 0],
            [0, -1, 0, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1],
        ]
    )

    # Rotate 90 degrees around the x-axis
    rotated_matrix = rotate_matrix(matrix, 180, "y")
    print("Rotated matrixs:\n", rotated_matrix)
