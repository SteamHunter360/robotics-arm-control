import math
import matplotlib.pyplot as plt

# Link lengths
L1 = 1.0
L2 = 0.75

# Workspace limits
max_reach = L1 + L2
min_reach = abs(L1 - L2)

fig, ax = plt.subplots(figsize=(7, 7))

# Outer reachable boundary
outer_circle = plt.Circle(
    (0, 0),
    max_reach,
    fill=False,
    linewidth=2,
    label="Maximum reach"
)

# Inner unreachable boundary
inner_circle = plt.Circle(
    (0, 0),
    min_reach,
    fill=False,
    linestyle="--",
    linewidth=2,
    label="Minimum reach"
)

ax.add_patch(outer_circle)
ax.add_patch(inner_circle)

# Plot base
ax.plot(0, 0, "ko", markersize=8, label="Base")

# Example arm pose
theta1 = math.radians(45)
theta2 = math.radians(30)

x0, y0 = 0, 0
x1 = L1 * math.cos(theta1)
y1 = L1 * math.sin(theta1)
x2 = x1 + L2 * math.cos(theta1 + theta2)
y2 = y1 + L2 * math.sin(theta1 + theta2)

ax.plot([x0, x1], [y0, y1], "o-", linewidth=3, label="Link 1")
ax.plot([x1, x2], [y1, y2], "o-", linewidth=3, label="Link 2")

# Formatting
ax.set_title("Robot Arm Workspace Visualisation")
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_aspect("equal")
ax.grid(True)
ax.legend()

plt.show()