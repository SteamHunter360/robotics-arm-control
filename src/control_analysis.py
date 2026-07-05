from src.metrics import (
    calculate_rms_error,
    calculate_max_tracking_error,
    calculate_overshoot,
    calculate_settling_time,
)


def analyse_closed_loop_response(
    time_values,
    actual_values,
    target_value,
    settling_tolerance=0.02,
):
    """
    Analyse the performance of a closed-loop step response.

    Returns final error, RMS tracking error, maximum tracking error,
    overshoot, and settling time.
    """

    if len(actual_values) == 0:
        raise ValueError("actual_values must not be empty.")

    if len(time_values) != len(actual_values):
        raise ValueError(
            "time_values and actual_values must have the same length."
        )

    final_value = actual_values[-1]

    final_error = abs(final_value - target_value)

    rms_error = calculate_rms_error(
        actual_values,
        target_value,
    )

    max_error = calculate_max_tracking_error(
        actual_values,
        target_value,
    )

    overshoot = calculate_overshoot(
        actual_values,
        target_value,
    )

    settling_time = calculate_settling_time(
        time_values,
        actual_values,
        target_value,
        tolerance=settling_tolerance,
    )

    return {
        "final_error": final_error,
        "rms_error": rms_error,
        "max_error": max_error,
        "overshoot": overshoot,
        "settling_time": settling_time,
    }