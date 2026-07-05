from src.pid_tuning import compare_pid_tunings
from src.pid_tuning_visualisation import plot_pid_tuning_comparison


def main():
    target_angle = 45.0

    tunings = {
        "Conservative": {
            "kp": 15.0,
            "ki": 0.0,
            "kd": 4.0,
        },
        "Balanced": {
            "kp": 30.0,
            "ki": 0.0,
            "kd": 5.0,
        },
        "Aggressive": {
            "kp": 60.0,
            "ki": 0.0,
            "kd": 6.0,
        },
    }

    results = compare_pid_tunings(
        tunings=tunings,
        target_angle=target_angle,
        initial_angle=0.0,
        duration=3.0,
        dt=0.01,
    )

    print("\nPID Tuning Comparison")

    for name, result in results.items():
        gains = result["gains"]
        metrics = result["metrics"]

        print(f"\n{name}")
        print(
            f"Kp={gains['kp']}, "
            f"Ki={gains['ki']}, "
            f"Kd={gains['kd']}"
        )

        print(f"Final error: {metrics['final_error']:.6f} degrees")
        print(f"RMS error: {metrics['rms_error']:.6f} degrees")
        print(f"Maximum error: {metrics['max_error']:.6f} degrees")
        print(f"Overshoot: {metrics['overshoot']:.6f} degrees")

        if metrics["settling_time"] is not None:
            print(f"Settling time: {metrics['settling_time']:.3f} s")
        else:
            print("Settling time: Not settled within simulation duration")

        print(
            f"RMS control effort: "
            f"{metrics['rms_control_effort']:.6f}"
        )

    plot_pid_tuning_comparison(
        results,
        save_path="images/pid_tuning_comparison.png",
    )


if __name__ == "__main__":
    main()