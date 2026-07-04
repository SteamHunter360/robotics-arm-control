import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from src.forward_kinematics import forward_kinematics


L1 = 1.0
L2 = 0.75

start_theta1 = 20
start_theta2 = 20

end_theta1 = 80
end_theta2 = 45

frames = 100


def generate_joint_trajectory(start_angle, end_angle, frames):
    return [
        start_angle + (end_angle - start_angle) * i / (frames - 1)
        for i in range(frames)
    ]


theta1_values = generate_joint_trajectory(start_theta1, end_theta1, frames)
theta2_values = generate_joint_trajectory(start_theta2, end_theta2, frames)

fig, ax = plt.subplots(figsize=(7, 7))

link1, = ax.plot([], [], "o-", linewidth=3, label="Link 1")
link2, = ax.plot([], [], "o-", linewidth=3, label="Link 2")
end_path, = ax.plot([], [], "--", linewidth=2, label="End-effector path")

path_x = []
path_y = []

ax.set_xlim(-1.8, 1.8)
ax.set_ylim(-1.8, 1.8)
ax.set_aspect("equal")
ax.grid(True)
ax.legend()
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.set_title("2-Link Robot Arm Trajectory Planning")


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
        f"Trajectory Planning\nθ1 = {theta1:.1f}°, θ2 = {theta2:.1f}°"
    )

    return link1, link2, end_path


animation = FuncAnimation(
    fig,
    update,
    frames=frames,
    interval=50,
    blit=False,
    repeat=False,
)

plt.show()