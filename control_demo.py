from src.closed_loop_simulation import simulate_joint_position_control
from src.control_analysis import analyse_closed_loop_response
from src.control_visualisation import plot_control_response


def main():
    target_angle = 45.0

    time_values, angle_values, control_values = (
        simulate_joint_position_control(
            target_angle=target_angle,
            initial_angle=0.0,
            duration=3.0,
            dt=0.01,
        )
    )

   results = analyse_closed_loop_response(
    time_values,
    angle_values,
    target_angle,
)

print(f"Overshoot: {results['overshoot']:.6f} degrees")

if results["settling_time"] is None:
    print("Settling time: Did not settle within simulation duration")
else:
    print(f"Settling time: {results['settling_time']:.3f} s")

    print("\nClosed-Loop Control Results")
    print(f"Final error: {results['final_error']:.6f} degrees")
    print(f"RMS error: {results['rms_error']:.6f} degrees")
    print(f"Maximum error: {results['max_error']:.6f} degrees")

    plot_control_response(
        time_values,
        angle_values,
        control_values,
        target_angle,
    )


if __name__ == "__main__":
    main()