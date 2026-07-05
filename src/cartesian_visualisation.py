import matplotlib.pyplot as plt
from src.metrics import calculate_position_error


def plot_cartesian_tracking_error(desired_path, actual_path, save_path=None):
    errors = []

    for desired, actual in zip(desired_path, actual_path):
        desired_x, desired_y = desired
        actual_x, actual_y = actual

        errors.append(
            calculate_position_error(
                actual_x,
                actual_y,
                desired_x,
                desired_y,
            )
        )

    fig, ax = plt.subplots(figsize=(9, 5))

    ax.plot(errors, linewidth=2, label="Cartesian Tracking Error")

    ax.set_title("Cartesian End-Effector Tracking Error")
    ax.set_xlabel("Trajectory Step")
    ax.set_ylabel("Position Error (m)")
    ax.grid(True)
    ax.legend()

    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches="tight")

    plt.show()

    return fig