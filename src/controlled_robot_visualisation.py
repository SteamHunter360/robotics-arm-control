import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


def animate_desired_vs_actual_paths(
    desired_path,
    actual_path,
    interval=50,
    save_path=None,
):
    if len(desired_path) != len(actual_path):
        raise ValueError("Desired and actual paths must have the same length.")

    fig, ax = plt.subplots(figsize=(7, 7))

    desired_line, = ax.plot([], [], "--", label="Desired Path")
    actual_line, = ax.plot([], [], "-", label="Actual Path")
    desired_point, = ax.plot([], [], "o", label="Desired End Effector")
    actual_point, = ax.plot([], [], "x", label="Actual End Effector")

    desired_x = []
    desired_y = []
    actual_x = []
    actual_y = []

    ax.set_xlim(-1.9, 1.9)
    ax.set_ylim(-1.9, 1.9)
    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend()
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_title("Desired vs Actual End-Effector Motion")

    def update(frame):
        dx, dy = desired_path[frame]
        ax_, ay = actual_path[frame]

        desired_x.append(dx)
        desired_y.append(dy)
        actual_x.append(ax_)
        actual_y.append(ay)

        desired_line.set_data(desired_x, desired_y)
        actual_line.set_data(actual_x, actual_y)
        desired_point.set_data([dx], [dy])
        actual_point.set_data([ax_], [ay])

        return desired_line, actual_line, desired_point, actual_point

    animation = FuncAnimation(
        fig,
        update,
        frames=len(desired_path),
        interval=interval,
        blit=False,
        repeat=False,
    )

    if save_path is not None:
        animation.save(
            save_path,
            writer=PillowWriter(fps=20),
        )

    plt.show()

    return animation