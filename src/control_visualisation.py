import matplotlib.pyplot as plt


def plot_control_response(
    time_values,
    angle_values,
    control_values,
    target_angle,
):
    """
    Plot closed-loop control-system performance.

    Generates three separate figures:
    1. Desired vs actual joint angle.
    2. Tracking error over time.
    3. Control effort over time.
    """

    tracking_errors = [
        target_angle - actual_angle
        for actual_angle in angle_values
    ]

    # Desired vs actual response
    plt.figure(figsize=(8, 5))

    plt.plot(
        time_values,
        angle_values,
        label="Actual Angle",
    )

    plt.axhline(
        y=target_angle,
        linestyle="--",
        label="Desired Angle",
    )

    plt.xlabel("Time (s)")
    plt.ylabel("Joint Angle (degrees)")
    plt.title("Closed-Loop Joint Position Response")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    # Tracking error
    plt.figure(figsize=(8, 5))

    plt.plot(
        time_values,
        tracking_errors,
        label="Tracking Error",
    )

    plt.axhline(
        y=0.0,
        linestyle="--",
    )

    plt.xlabel("Time (s)")
    plt.ylabel("Tracking Error (degrees)")
    plt.title("Joint Tracking Error")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    # Control effort
    plt.figure(figsize=(8, 5))

    plt.plot(
        time_values,
        control_values,
        label="Control Input",
    )

    plt.xlabel("Time (s)")
    plt.ylabel("Control Input / Torque")
    plt.title("PID Control Effort")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.show()