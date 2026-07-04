import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from src.forward_kinematics import forward_kinematics


def generate_joint_trajectory(start_angle, end_angle, frames):
    if frames < 2:
        raise ValueError("frames must be at least 2.")

    return [
        start_angle + (end_angle - start_angle) * i / (frames - 1)
        for i in range(frames)
    ]


def animate_joint_trajectory(
    theta1_values,
    theta2_values,
    L1=1.0,
    L2=0.75,
    interval=50,
):
    if len(theta1_values) != len(theta2_values):
        raise ValueError("Joint trajectories must have the same length.")

    fig, ax = plt.subplots(figsize=(7, 7))

    link1, = ax.plot([], [], "o-", linewidth=3, label="Link 1")
    link2, = ax.plot([], [], "o-", linewidth=3, label="Link 2")
    end_path, = ax.plot([], [], "--", linewidth=2, label="End-effector path")

    path_x = []
    path_y = []

    reach = L1 + L2

    ax.set_xlim(-reach - 0.1, reach + 0.1)
    ax.set_ylim(-reach - 0.1, reach + 0.1)
    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend()
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")

    def update(frame):
        theta1 = theta1_values[frame]
        theta2 = theta2_values[frame]

        points = forward_kinematics(theta1, theta2, L1, L2)

        x0, y0 = points["base"]
        x1, y1 = points["joint_1"]
        x2, y2 = points["end_effector"]

        link1.set_data([x0, x1], [y0, y1])
        link2.set_data([x1, x2], [y1, y2])

        path_x.append(x2)
        path_y.append(y2)
        end_path.set_data(path_x, path_y)

        ax.set_title(
            f"Robot Arm Trajectory\n"
            f"θ1 = {theta1:.1f}°, θ2 = {theta2:.1f}°"
        )

        return link1, link2, end_path

    animation = FuncAnimation(
        fig,
        update,
        frames=len(theta1_values),
        interval=interval,
        blit=False,
        repeat=False,
    )

    plt.show()

    return animation


if __name__ == "__main__":
    theta1_values = generate_joint_trajectory(20, 80, 100)
    theta2_values = generate_joint_trajectory(20, 45, 100)

    animate_joint_trajectory(theta1_values, theta2_values)