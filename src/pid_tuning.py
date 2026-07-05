from src.closed_loop_simulation import simulate_joint_position_control
from src.control_analysis import analyse_closed_loop_response


def calculate_rms_control_effort(control_values):
    if len(control_values) == 0:
        return 0.0

    mean_squared_effort = sum(
        control_input**2
        for control_input in control_values
    ) / len(control_values)

    return mean_squared_effort**0.5


def compare_pid_tunings(
    tunings,
    target_angle=45.0,
    initial_angle=0.0,
    duration=3.0,
    dt=0.01,
):
    """
    Simulate and quantitatively compare multiple PID tunings.

    Args:
        tunings:
            Dictionary mapping tuning names to dictionaries containing
            kp, ki, and kd values.

    Returns:
        Dictionary containing performance metrics and simulation data
        for every controller tuning.
    """

    comparison_results = {}

    for tuning_name, gains in tunings.items():
        time_values, angle_values, control_values = (
            simulate_joint_position_control(
                target_angle=target_angle,
                initial_angle=initial_angle,
                duration=duration,
                dt=dt,
                kp=gains["kp"],
                ki=gains["ki"],
                kd=gains["kd"],
            )
        )

        metrics = analyse_closed_loop_response(
            time_values,
            angle_values,
            target_angle,
        )

        metrics["rms_control_effort"] = calculate_rms_control_effort(
            control_values
        )

        comparison_results[tuning_name] = {
            "gains": gains,
            "metrics": metrics,
            "time_values": time_values,
            "angle_values": angle_values,
            "control_values": control_values,
        }

    return comparison_results