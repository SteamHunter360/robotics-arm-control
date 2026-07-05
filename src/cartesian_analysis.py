from src.forward_kinematics import forward_kinematics
from src.metrics import (
    calculate_position_error,
    calculate_path_length,
    calculate_max_step_distance,
)


def calculate_end_effector_path(theta1_values, theta2_values, L1=1.0, L2=0.75):
    if len(theta1_values) != len(theta2_values):
        raise ValueError("Joint angle trajectories must have the same length.")

    path = []

    for theta1, theta2 in zip(theta1_values, theta2_values):
        points = forward_kinematics(theta1, theta2, L1, L2)
        path.append(points["end_effector"])

    return path


def analyse_cartesian_tracking(desired_path, actual_path):
    if len(desired_path) != len(actual_path):
        raise ValueError("Desired and actual paths must have the same length.")

    errors = []

    for desired, actual in zip(desired_path, actual_path):
        desired_x, desired_y = desired
        actual_x, actual_y = actual

        error = calculate_position_error(
            actual_x,
            actual_y,
            desired_x,
            desired_y,
        )

        errors.append(error)

    final_error = errors[-1]
    max_error = max(errors)
    rms_error = (sum(error**2 for error in errors) / len(errors)) ** 0.5

    return {
        "final_error": final_error,
        "max_error": max_error,
        "rms_error": rms_error,
        "desired_path_length": calculate_path_length(desired_path),
        "actual_path_length": calculate_path_length(actual_path),
        "desired_max_step": calculate_max_step_distance(desired_path),
        "actual_max_step": calculate_max_step_distance(actual_path),
    }