from src.pid_tuning import compare_pid_tunings


def main():
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

    results = compare_pid_tunings(tunings)

    print("\nPID Tuning Comparison")

    for tuning_name, result in results.items():
        gains = result["gains"]
        metrics = result["metrics"]

        print(f"\n{tuning_name}")
        print(
            f"Kp={gains['kp']}, "
            f"Ki={gains['ki']}, "
            f"Kd={gains['kd']}"
        )
        print(f"Final error: {metrics['final_error']:.6f} degrees")
        print(f"RMS error: {metrics['rms_error']:.6f} degrees")
        print(f"Maximum error: {metrics['max_error']:.6f} degrees")
        print(f"Overshoot: {metrics['overshoot']:.6f} degrees")

        if metrics["settling_time"] is None:
            print("Settling time: Did not settle")
        else:
            print(f"Settling time: {metrics['settling_time']:.3f} s")

        print(
            "RMS control effort: "
            f"{metrics['rms_control_effort']:.6f}"
        )


if __name__ == "__main__":
    main()