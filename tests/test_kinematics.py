import math
import pytest

from src.forward_kinematics import forward_kinematics
from src.inverse_kinematics import inverse_kinematics
from src.trajectory_planning import generate_joint_trajectory
from src.metrics import (
    calculate_position_error,
    calculate_path_length,
    calculate_max_step_distance,
    calculate_rms_error,
    calculate_max_tracking_error,
)
from src.pid_controller import PIDController
from src.joint_simulation import JointSimulation
from src.closed_loop_simulation import simulate_joint_position_control


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

    
def test_generate_joint_trajectory_rejects_invalid_frames():
    with pytest.raises(ValueError):
        generate_joint_trajectory(20, 80, 1)

def test_calculate_path_length_known_path():
    points = [
        (0, 0),
        (3, 4),
        (6, 8),
    ]

    path_length = calculate_path_length(points)

    assert path_length == pytest.approx(10.0)

def test_calculate_max_step_distance_known_path():
    points = [
        (0, 0),
        (3, 4),
        (9, 12),
    ]

    max_step_distance = calculate_max_step_distance(points)

    assert max_step_distance == pytest.approx(10.0)


def test_pid_controller_generates_positive_output_for_positive_error():
    controller = PIDController(
        kp=2.0,
        ki=0.0,
        kd=0.0,
        dt=0.1,
    )

    output = controller.update(
        setpoint=10.0,
        measurement=8.0,
    )

    assert output == pytest.approx(4.0)

def test_pid_controller_generates_negative_output_for_negative_error():
    controller = PIDController(
        kp=2.0,
        ki=0.0,
        kd=0.0,
        dt=0.1,
    )

    output = controller.update(
        setpoint=8.0,
        measurement=10.0,
    )

    assert output == pytest.approx(-4.0)


def test_pid_controller_output_limit():
    controller = PIDController(
        kp=10.0,
        ki=0.0,
        kd=0.0,
        dt=0.1,
        output_limit=5.0,
    )

    output = controller.update(
        setpoint=10.0,
        measurement=0.0,
    )

    assert output == pytest.approx(5.0)


def test_pid_controller_reset():
    controller = PIDController(
        kp=1.0,
        ki=1.0,
        kd=0.0,
        dt=0.1,
    )

    controller.update(setpoint=10.0, measurement=0.0)
    controller.reset()

    assert controller.integral == pytest.approx(0.0)
    assert controller.previous_error == pytest.approx(0.0)

def test_joint_simulation_positive_torque_increases_angle():
    joint = JointSimulation(
        initial_angle=0.0,
        initial_velocity=0.0,
        inertia=1.0,
        damping=0.0,
        dt=0.1,
    )

    angle = joint.update(control_input=1.0)

    assert angle > 0.0


def test_joint_simulation_zero_input_stays_at_rest():
    joint = JointSimulation(
        initial_angle=0.0,
        initial_velocity=0.0,
        inertia=1.0,
        damping=1.0,
        dt=0.1,
    )

    angle = joint.update(control_input=0.0)

    assert angle == pytest.approx(0.0)
    assert joint.velocity == pytest.approx(0.0)


def test_joint_simulation_rejects_invalid_inertia():
    with pytest.raises(ValueError):
        JointSimulation(inertia=0.0)

def test_closed_loop_joint_converges_toward_target():
    target_angle = 45.0

    time_values, angle_values, control_values = simulate_joint_position_control(
        target_angle=target_angle,
        initial_angle=0.0,
        duration=3.0,
        dt=0.01,
    )

    final_angle = angle_values[-1]
    final_error = abs(final_angle - target_angle)

    assert final_error < 5.0
    assert len(time_values) == len(angle_values)
    assert len(angle_values) == len(control_values)

def test_calculate_rms_error_known_values():
    actual_values = [1, 2, 3]
    target_value = 2

    rms_error = calculate_rms_error(actual_values, target_value)

    assert rms_error == pytest.approx((2 / 3) ** 0.5)

def test_calculate_max_tracking_error_known_values():
    actual_values = [1, 2, 3]
    target_value = 2

    max_error = calculate_max_tracking_error(
        actual_values,
        target_value,
    )

    assert max_error == pytest.approx(1.0)