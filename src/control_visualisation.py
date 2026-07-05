import matplotlib.pyplot as plt


def plot_control_response(
    time_values,
    angle_values,
    control_values,
    target_angle,
    save_path=None,
):
    """
    Plot the closed-loop joint response and control input.

    Args:
        time_values: Simulation time samples.
        angle_values: Actual joint-angle samples.
        control_values: Controller output samples.
        target_angle: Desired joint angle.
        save_path: Optional path used to save the generated figure.
    """

    fig, angle_ax = plt.subplots(figsize=(9, 6))

    angle_ax.plot(
        time_values,
        angle_values,
        linewidth=2,
        label="Actual Joint Angle",
    )

    angle_ax.axhline(
        target_angle,
        linestyle="--",
        linewidth=2,
        label="Target Joint Angle",
    )

    angle_ax.set_xlabel("Time (s)")
    angle_ax.set_ylabel("Joint Angle (degrees)")
    angle_ax.set_title("Closed-Loop PID Joint Position Control")
    angle_ax.grid(True)
    angle_ax.legend(loc="upper left")

    control_ax = angle_ax.twinx()

    control_ax.plot(
        time_values,
        control_values,
        linewidth=1.5,
        alpha=0.7,
        label="Control Input",
    )

    control_ax.set_ylabel("Control Input")

    fig.tight_layout()

    if save_path is not None:
        fig.savefig(
            save_path,
            dpi=300,
            bbox_inches="tight",
        )

    plt.show()

    return fig