from src.cartesian_analysis import (
    calculate_end_effector_path,
    analyse_cartesian_tracking,
)


def test_calculate_end_effector_path_returns_matching_length():
    theta1_values = [0, 45, 90]
    theta2_values = [0, 30, 0]

    path = calculate_end_effector_path(theta1_values, theta2_values)

    assert len(path) == 3


def test_analyse_cartesian_tracking_zero_error():
    desired_path = [(0, 0), (1, 1), (2, 2)]
    actual_path = [(0, 0), (1, 1), (2, 2)]

    results = analyse_cartesian_tracking(desired_path, actual_path)

    assert results["final_error"] == 0.0
    assert results["max_error"] == 0.0
    assert results["rms_error"] == 0.0