import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Link lengths
L1 = 1.0
L2 = 0.75

# Initial joint angles
theta1_initial = 45
theta2_initial = 30


def calculate_joint_positions(theta1_deg, theta2_deg):
    """Calculate base, joint, and end-effector positions."""

    theta1 = math.radians(theta1_deg)
    theta2 = math.radians(theta2_deg)

    x0, y0 = 0, 0

    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)

    x2 = x1 + L2 * math.cos(theta1 + theta2)
    y2 = y1 + L2 * math.sin(theta1 + theta2)

    return x0, y0, x1, y1, x2, y2


# Initial positions
x0, y0, x1, y1, x2, y2 = calculate_joint_positions(
    theta1_initial,
    theta2_initial
)

# Create figure
fig, ax = plt.subplots(figsize=(8, 6))
plt.subplots_adjust(bottom=0.25)

# Plot links
link1, = ax.plot([x0, x1], [y0, y1], "o-", linewidth=3, label="Link 1")
link2, = ax.plot([x1, x2], [y1, y2], "o-", linewidth=3, label="Link 2")

# Plot joints
base_point, = ax.plot(x0, y0, "o", color="black", markersize=10)
joint1_point, = ax.plot(x1, y1, "o", color="green", markersize=10)
end_effector_point, = ax.plot(x2, y2, "o", color="red", markersize=10)

# Title and formatting
ax.set_title("Interactive 2-Link Robot Arm", fontsize=14, fontweight="bold")
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.grid(True)
ax.axis("equal")
ax.set_xlim(-1.8, 1.8)
ax.set_ylim(-1.8, 1.8)
ax.legend()

# Slider axes
theta1_slider_ax = plt.axes([0.20, 0.12, 0.65, 0.03])
theta2_slider_ax = plt.axes([0.20, 0.06, 0.65, 0.03])

theta1_slider = Slider(
    ax=theta1_slider_ax,
    label="θ1",
    valmin=-180,
    valmax=180,
    valinit=theta1_initial
)

theta2_slider = Slider(
    ax=theta2_slider_ax,
    label="θ2",
    valmin=-180,
    valmax=180,
    valinit=theta2_initial
)


def update_plot(val):
    """Update the robot arm when sliders are moved."""

    theta1_deg = theta1_slider.val
    theta2_deg = theta2_slider.val

    x0, y0, x1, y1, x2, y2 = calculate_joint_positions(
        theta1_deg,
        theta2_deg
    )

    link1.set_data([x0, x1], [y0, y1])
    link2.set_data([x1, x2], [y1, y2])

    base_point.set_data([x0], [y0])
    joint1_point.set_data([x1], [y1])
    end_effector_point.set_data([x2], [y2])

    ax.set_title(
        f"Interactive 2-Link Robot Arm\nθ1 = {theta1_deg:.1f}°, θ2 = {theta2_deg:.1f}°",
        fontsize=14,
        fontweight="bold"
    )

    fig.canvas.draw_idle()


theta1_slider.on_changed(update_plot)
theta2_slider.on_changed(update_plot)

plt.show()