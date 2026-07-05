import pytest

from src.pid_controller import PIDController
from src.joint_simulation import JointSimulation
from src.closed_loop_simulation import simulate_joint_position_control
from src.control_analysis import analyse_closed_loop_response
from src.pid_tuning import (
    calculate_rms_control_effort,
    compare_pid_tunings,
)
from src.two_joint_control import simulate_two_joint_position_control
from src.two_joint_trajectory_tracking import simulate_two_joint_trajectory_tracking
from src.trajectory_planning import generate_joint_trajectory


def test_pid_controller_generates_positive_output_for_positive_error():
    controller = PIDController(kp=2.0, ki=0.0, kd=0.0, dt=0.1)

    output = controller.update(setpoint=10.0, measurement=8.0)

    assert output == pytest.approx(4.0)


def test_pid_controller_generates_negative_output_for_negative_error():
    controller = PIDController(kp=2.0, ki=0.0, kd=0.0, dt=0.1)

    output = controller.update(setpoint=8.0, measurement=10.0)

    assert output == pytest.approx(-4.0)


def test_pid_controller_output_limit():
    controller = PIDController(
        kp=10.0,
        ki=0.0,
        kd=0.0,
        dt=0.1,
        output_limit=5.0,
    )

    output = controller.update(setpoint=10.0, measurement=0.0)

    assert output == pytest.approx(5.0)


def test_pid_controller_reset():
    controller = PIDController(kp=1.0, ki=1.0, kd=0.0, dt=0.1)

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


def test_analyse_closed_loop_response_returns_expected_metrics():
    actual_values = [0, 5, 10]
    target_value = 10
    time_values = [0, 1, 2]

    results = analyse_closed_loop_response(
        time_values,
        actual_values,
        target_value,
    )

    assert results["final_error"] == pytest.approx(0.0)
    assert results["rms_error"] == pytest.approx(((100 + 25 + 0) / 3) ** 0.5)
    assert results["max_error"] == pytest.approx(10.0)
    assert results["overshoot"] == pytest.approx(0.0)
    assert results["settling_time"] == pytest.approx(2.0)


def test_calculate_rms_control_effort_known_values():
    control_values = [3.0, 4.0]

    rms_effort = calculate_rms_control_effort(control_values)

    assert rms_effort == pytest.approx((25 / 2) ** 0.5)


def test_compare_pid_tunings_returns_all_tunings():
    tunings = {
        "Tuning A": {"kp": 15.0, "ki": 0.0, "kd": 4.0},
        "Tuning B": {"kp": 30.0, "ki": 0.0, "kd": 5.0},
    }

    results = compare_pid_tunings(tunings, duration=1.0, dt=0.01)

    assert set(results.keys()) == {"Tuning A", "Tuning B"}

    for result in results.values():
        assert "gains" in result
        assert "metrics" in result
        assert "time_values" in result
        assert "angle_values" in result
        assert "control_values" in result

        assert "final_error" in result["metrics"]
        assert "rms_error" in result["metrics"]
        assert "overshoot" in result["metrics"]
        assert "settling_time" in result["metrics"]
        assert "rms_control_effort" in result["metrics"]


def test_two_joint_position_control_converges_toward_targets():
    target_theta1 = 60.0
    target_theta2 = 45.0

    results = simulate_two_joint_position_control(
        target_theta1=target_theta1,
        target_theta2=target_theta2,
        initial_theta1=20.0,
        initial_theta2=20.0,
        duration=3.0,
        dt=0.01,
    )

    final_theta1 = results["theta1_values"][-1]
    final_theta2 = results["theta2_values"][-1]

    assert abs(final_theta1 - target_theta1) < 5.0
    assert abs(final_theta2 - target_theta2) < 5.0

    assert len(results["time_values"]) == len(results["theta1_values"])
    assert len(results["time_values"]) == len(results["theta2_values"])


def test_two_joint_trajectory_tracking_returns_matching_lengths():
    desired_theta1 = generate_joint_trajectory(20, 60, 100)
    desired_theta2 = generate_joint_trajectory(20, 45, 100)

    results = simulate_two_joint_trajectory_tracking(
        desired_theta1,
        desired_theta2,
    )

    assert len(results["time_values"]) == 100
    assert len(results["actual_theta1_values"]) == 100
    assert len(results["actual_theta2_values"]) == 100
    assert len(results["control_1_values"]) == 100
    assert len(results["control_2_values"]) == 100