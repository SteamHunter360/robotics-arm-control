import pytest

from src.metrics import (
    calculate_position_error,
    calculate_path_length,
    calculate_max_step_distance,
    calculate_rms_error,
    calculate_max_tracking_error,
    calculate_overshoot,
    calculate_settling_time,
)


def test_calculate_position_error_zero_error():
    error = calculate_position_error(0.9, 1.2, 0.9, 1.2)

    assert error == pytest.approx(0.0)


def test_calculate_position_error_known_distance():
    error = calculate_position_error(3, 4, 0, 0)

    assert error == pytest.approx(5.0)


def test_calculate_path_length_known_path():
    points = [(0, 0), (3, 4), (6, 8)]

    path_length = calculate_path_length(points)

    assert path_length == pytest.approx(10.0)


def test_calculate_max_step_distance_known_path():
    points = [(0, 0), (3, 4), (9, 12)]

    max_step_distance = calculate_max_step_distance(points)

    assert max_step_distance == pytest.approx(10.0)


def test_calculate_rms_error_known_values():
    actual_values = [1, 2, 3]
    target_value = 2

    rms_error = calculate_rms_error(actual_values, target_value)

    assert rms_error == pytest.approx((2 / 3) ** 0.5)


def test_calculate_max_tracking_error_known_values():
    actual_values = [1, 2, 3]
    target_value = 2

    max_error = calculate_max_tracking_error(actual_values, target_value)

    assert max_error == pytest.approx(1.0)


def test_calculate_overshoot_known_values():
    actual_values = [0, 5, 12, 10]
    target_value = 10

    overshoot = calculate_overshoot(actual_values, target_value)

    assert overshoot == pytest.approx(2.0)


def test_calculate_settling_time_known_values():
    time_values = [0, 1, 2, 3, 4]
    actual_values = [0, 8, 9.7, 10.1, 10.0]
    target_value = 10

    settling_time = calculate_settling_time(
        time_values,
        actual_values,
        target_value,
        tolerance=0.05,
    )

    assert settling_time == pytest.approx(2)