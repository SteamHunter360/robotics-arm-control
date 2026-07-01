import math
import matplotlib.pyplot as plt

# Link lengths
L1 = 1.0
L2 = 0.75

# -------------------------
# Target Position
# -------------------------

target_x = float(input("Enter target X coordinate: "))
target_y = float(input("Enter target Y coordinate: "))

# -------------------------
# Inverse Kinematics
# -------------------------

distance_squared = target_x**2 + target_y**2

cos_theta2 = (
    distance_squared
    - L1**2
    - L2**2
) / (2 * L1 * L2)

if abs(cos_theta2) > 1:

    print("Target is outside the robot workspace.")
    exit()

theta2 = math.acos(cos_theta2)

theta1 = math.atan2(target_y, target_x) - math.atan2(
    L2 * math.sin(theta2),
    L1 + L2 * math.cos(theta2)
)

# Convert to degrees

theta1_deg = math.degrees(theta1)
theta2_deg = math.degrees(theta2)

print()

print(f"Theta1 = {theta1_deg:.2f}°")
print(f"Theta2 = {theta2_deg:.2f}°")

# -------------------------
# Forward Kinematics
# -------------------------

x0, y0 = 0, 0

x1 = L1 * math.cos(theta1)
y1 = L1 * math.sin(theta1)

x2 = x1 + L2 * math.cos(theta1 + theta2)
y2 = y1 + L2 * math.sin(theta1 + theta2)

# -------------------------
# Plot
# -------------------------

plt.figure(figsize=(7,7))

plt.plot(
    [x0, x1],
    [y0, y1],
    "o-",
    linewidth=3,
    label="Link 1"
)

plt.plot(
    [x1, x2],
    [y1, y2],
    "o-",
    linewidth=3,
    label="Link 2"
)

plt.scatter(
    target_x,
    target_y,
    marker="x",
    s=120,
    label="Target"
)

plt.title("Inverse Kinematics Solution")

plt.xlabel("X")

plt.ylabel("Y")

plt.grid(True)

plt.axis("equal")

plt.legend()

plt.show()