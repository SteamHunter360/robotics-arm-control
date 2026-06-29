import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

L1 = 1.0
L2 = 0.75
frames = 60

current_theta1 = math.radians(45)
current_theta2 = math.radians(30)


def forward_kinematics(theta1, theta2):
    x0, y0 = 0, 0
    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)
    x2 = x1 + L2 * math.cos(theta1 + theta2)
    y2 = y1 + L2 * math.sin(theta1 + theta2)
    return x0, y0, x1, y1, x2, y2


def inverse_kinematics(target_x, target_y):
    distance_squared = target_x**2 + target_y**2

    cos_theta2 = (distance_squared - L1**2 - L2**2) / (2 * L1 * L2)

    if abs(cos_theta2) > 1:
        return None

    theta2 = math.acos(cos_theta2)

    theta1 = math.atan2(target_y, target_x) - math.atan2(
        L2 * math.sin(theta2),
        L1 + L2 * math.cos(theta2)
    )

    return theta1, theta2


fig, ax = plt.subplots(figsize=(7, 7))

x0, y0, x1, y1, x2, y2 = forward_kinematics(current_theta1, current_theta2)

link1, = ax.plot([x0, x1], [y0, y1], "o-", linewidth=3, label="Link 1")
link2, = ax.plot([x1, x2], [y1, y2], "o-", linewidth=3, label="Link 2")
target_point, = ax.plot([], [], "rx", markersize=12, label="Target")
path_line, = ax.plot([], [], "--", linewidth=2, label="End-effector path")

path_x = []
path_y = []

ax.set_xlim(-1.8, 1.8)
ax.set_ylim(-1.8, 1.8)
ax.set_aspect("equal")
ax.grid(True)
ax.legend()
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.set_title("Click-to-Move Robot Arm\nClick inside the reachable workspace")


def update_arm(theta1, theta2):
    x0, y0, x1, y1, x2, y2 = forward_kinematics(theta1, theta2)

    link1.set_data([x0, x1], [y0, y1])
    link2.set_data([x1, x2], [y1, y2])

    path_x.append(x2)
    path_y.append(y2)
    path_line.set_data(path_x, path_y)

    ax.set_title(
        f"Click-to-Move Robot Arm\nθ1 = {math.degrees(theta1):.1f}°, θ2 = {math.degrees(theta2):.1f}°"
    )


animation = None


def on_click(event):
    global current_theta1, current_theta2, animation

    if event.inaxes != ax:
        return

    target_x = event.xdata
    target_y = event.ydata

    result = inverse_kinematics(target_x, target_y)

    if result is None:
        ax.set_title("Target outside reachable workspace")
        fig.canvas.draw_idle()
        return

    target_theta1, target_theta2 = result

    target_point.set_data([target_x], [target_y])

    theta1_values = [
        current_theta1 + (target_theta1 - current_theta1) * i / (frames - 1)
        for i in range(frames)
    ]

    theta2_values = [
        current_theta2 + (target_theta2 - current_theta2) * i / (frames - 1)
        for i in range(frames)
    ]

    def animate(frame):
        theta1 = theta1_values[frame]
        theta2 = theta2_values[frame]
        update_arm(theta1, theta2)
        return link1, link2, target_point, path_line

    animation = FuncAnimation(
        fig,
        animate,
        frames=frames,
        interval=40,
        blit=False,
        repeat=False
    )

    current_theta1 = target_theta1
    current_theta2 = target_theta2

    fig.canvas.draw_idle()


fig.canvas.mpl_connect("button_press_event", on_click)

plt.show()