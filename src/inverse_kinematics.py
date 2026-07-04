import math
import matplotlib.pyplot as plt

from src.forward_kinematics import forward_kinematics


def inverse_kinematics(target_x, target_y, L1=1.0, L2=0.75):
    distance_squared = target_x**2 + target_y**2

    cos_theta2 = (distance_squared - L1**2 - L2**2) / (2 * L1 * L2)

    if abs(cos_theta2) > 1:
        raise ValueError("Target is outside the robot workspace.")

    theta2 = math.acos(cos_theta2)

    theta1 = math.atan2(target_y, target_x) - math.atan2(
        L2 * math.sin(theta2),
        L1 + L2 * math.cos(theta2),
    )

    return math.degrees(theta1), math.degrees(theta2)


def plot_inverse_kinematics_solution(target_x, target_y, L1=1.0, L2=0.75):
    theta1_deg, theta2_deg = inverse_kinematics(target_x, target_y, L1, L2)

    points = forward_kinematics(theta1_deg, theta2_deg, L1, L2)

    x0, y0 = points["base"]
    x1, y1 = points["joint_1"]
    x2, y2 = points["end_effector"]

    plt.figure(figsize=(7, 7))

    plt.plot([x0, x1], [y0, y1], "o-", linewidth=3, label="Link 1")
    plt.plot([x1, x2], [y1, y2], "o-", linewidth=3, label="Link 2")
    plt.scatter(target_x, target_y, marker="x", s=120, label="Target")

    plt.title("Inverse Kinematics Solution")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.grid(True)
    plt.axis("equal")
    plt.legend()
    plt.show()

    return theta1_deg, theta2_deg, (x2, y2)


if __name__ == "__main__":
    target_x = float(input("Enter target X coordinate: "))
    target_y = float(input("Enter target Y coordinate: "))

    theta1, theta2, end_effector = plot_inverse_kinematics_solution(
        target_x,
        target_y,
    )

    print(f"Theta1 = {theta1:.2f}°")
    print(f"Theta2 = {theta2:.2f}°")
    print(f"End-effector position = ({end_effector[0]:.3f}, {end_effector[1]:.3f})")