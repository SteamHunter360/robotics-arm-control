import math
import pytest

from src.forward_kinematics import forward_kinematics
from src.inverse_kinematics import inverse_kinematics
from src.trajectory_planning import generate_joint_trajectory
from src.metrics import calculate_position_error


def test_forward_kinematics_zero_angles():
    points = forward_kinematics(0, 0)
    x, y = points["end_effector"]

    assert x == pytest.approx(1.75)
    assert y == pytest.approx(0.0)


def test_forward_kinematics_vertical_arm():
    points = forward_kinematics(90, 0)
    x, y = points["end_effector"]

    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(1.75)


def test_forward_kinematics_folded_arm():
    points = forward_kinematics(0, 180)
    x, y = points["end_effector"]

    assert x == pytest.approx(0.25)
    assert y == pytest.approx(0.0, abs=1e-9)


def test_forward_kinematics_known_position():
    points = forward_kinematics(
        theta1_deg=45,
        theta2_deg=30,
        L1=1.0,
        L2=0.75,
    )

    x, y = points["end_effector"]

    assert x == pytest.approx(0.901221, abs=1e-6)
    assert y == pytest.approx(1.431551, abs=1e-6)


def test_inverse_kinematics_reaches_target():
    target_x = 0.9
    target_y = 1.2

    theta1, theta2 = inverse_kinematics(target_x, target_y)
    points = forward_kinematics(theta1, theta2)

    x, y = points["end_effector"]

    assert x == pytest.approx(target_x, abs=1e-6)
    assert y == pytest.approx(target_y, abs=1e-6)


def test_inverse_kinematics_rejects_unreachable_target():
    with pytest.raises(ValueError):
        inverse_kinematics(3.0, 0.0)


def test_generate_joint_trajectory_start_and_end_values():
    trajectory = generate_joint_trajectory(20, 80, 100)

    assert trajectory[0] == pytest.approx(20)
    assert trajectory[-1] == pytest.approx(80)


def test_generate_joint_trajectory_number_of_frames():
    trajectory = generate_joint_trajectory(20, 80, 100)

    assert len(trajectory) == 100


def test_integrated_ik_trajectory_fk_pipeline():
    target_x = 0.9
    target_y = 1.2

    start_theta1 = 20
    start_theta2 = 20

    target_theta1, target_theta2 = inverse_kinematics(target_x, target_y)

    theta1_trajectory = generate_joint_trajectory(
        start_theta1,
        target_theta1,
        100,
    )

    theta2_trajectory = generate_joint_trajectory(
        start_theta2,
        target_theta2,
        100,
    )

    for theta1, theta2 in zip(theta1_trajectory, theta2_trajectory):
        points = forward_kinematics(theta1, theta2)
        x, y = points["end_effector"]

        assert math.isfinite(x)
        assert math.isfinite(y)

    final_points = forward_kinematics(
        theta1_trajectory[-1],
        theta2_trajectory[-1],
    )

    final_x, final_y = final_points["end_effector"]

    final_error = math.hypot(
        final_x - target_x,
        final_y - target_y,
    )

    assert final_error < 1e-6


def test_calculate_position_error_zero_error():
    error = calculate_position_error(0.9, 1.2, 0.9, 1.2)

    assert error == pytest.approx(0.0)


def test_calculate_position_error_known_distance():
    error = calculate_position_error(3, 4, 0, 0)

    assert error == pytest.approx(5.0)