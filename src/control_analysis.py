from src.metrics import (
    calculate_rms_error,
    calculate_max_tracking_error,
)


def analyse_closed_loop_response(actual_values, target_value):
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

    return {
        "final_error": final_error,
        "rms_error": rms_error,
        "max_error": max_error,
    }