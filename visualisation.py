import math
import matplotlib.pyplot as plt

# Link lengths
L1 = 1.0
L2 = 0.75

# Joint angles in degrees
theta1 = math.radians(45)
theta2 = math.radians(30)

# Base coordinates
x0, y0 = 0, 0

# Joint 1 coordinates
x1 = L1 * math.cos(theta1)
y1 = L1 * math.sin(theta1)

# End-effector coordinates
x2 = x1 + L2 * math.cos(theta1 + theta2)
y2 = y1 + L2 * math.sin(theta1 + theta2)

# Create figure
fig, ax = plt.subplots(figsize=(8, 6))

# Plot links with different colours
ax.plot([x0, x1], [y0, y1], marker="o", linewidth=3, label="Link 1")
ax.plot([x1, x2], [y1, y2], marker="o", linewidth=3, label="Link 2")

# Plot joints with custom colours
ax.plot(x0, y0, "o", color="black", markersize=10)
ax.plot(x1, y1, "o", color="green", markersize=10)
ax.plot(x2, y2, "o", color="red", markersize=10)

# Label the joints and display coordinates
ax.annotate(
    f"Base\n({x0:.2f}, {y0:.2f})",
    xy=(x0, y0),
    xytext=(-0.35, -0.15),
    arrowprops=dict(arrowstyle="->"),
    fontsize=10,
)

ax.annotate(
    f"Joint 1\n({x1:.2f}, {y1:.2f})",
    xy=(x1, y1),
    xytext=(x1 + 0.15, y1 + 0.10),
    arrowprops=dict(arrowstyle="->"),
    fontsize=10,
)

ax.annotate(
    f"End Effector\n({x2:.2f}, {y2:.2f})",
    xy=(x2, y2),
    xytext=(x2 + 0.15, y2 + 0.10),
    arrowprops=dict(arrowstyle="->"),
    fontsize=10,
)

# Formatting
ax.set_title("2-Link Robot Arm", fontsize=14, fontweight="bold")
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.grid(True)
ax.axis("equal")
ax.legend()

# Save figure
plt.savefig("images/robot_arm_visualisation.png", dpi=300)

# Show figure
plt.show()