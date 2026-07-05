import pytest

from src.trajectory_planning import generate_joint_trajectory
from src.two_joint_trajectory_tracking import simulate_two_joint_trajectory_tracking
from src.cartesian_analysis import (
    calculate_end_effector_path,
    analyse_cartesian_tracking,
)
from src.control_analysis import analyse_closed_loop_response


def test_two_joint_trajectory_tracking_rejects_mismatched_lengths():
    desired_theta1 = [20, 30, 40]
    desired_theta2 = [20, 30]

    with pytest.raises(ValueError):
        simulate_two_joint_trajectory_tracking(
            desired_theta1,
            desired_theta2,
        )


def test_calculate_end_effector_path_rejects_mismatched_lengths():
    theta1_values = [0, 45, 90]
    theta2_values = [0, 30]

    with pytest.raises(ValueError):
        calculate_end_effector_path(theta1_values, theta2_values)


def test_analyse_cartesian_tracking_rejects_mismatched_lengths():
    desired_path = [(0, 0), (1, 1)]
    actual_path = [(0, 0)]

    with pytest.raises(ValueError):
        analyse_cartesian_tracking(desired_path, actual_path)


def test_control_analysis_rejects_empty_actual_values():
    with pytest.raises(ValueError):
        analyse_closed_loop_response([], [], 10)


def test_control_analysis_rejects_mismatched_lengths():
    time_values = [0, 1, 2]
    actual_values = [0, 5]

    with pytest.raises(ValueError):
        analyse_closed_loop_response(
            time_values,
            actual_values,
            10,
        )


def test_generate_joint_trajectory_rejects_zero_frames():
    with pytest.raises(ValueError):
        generate_joint_trajectory(20, 80, 0)