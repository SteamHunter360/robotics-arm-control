from src.inverse_kinematics import inverse_kinematics
from src.forward_kinematics import forward_kinematics
from src.trajectory_planning import generate_joint_trajectory


def main():
    target_x = float(input("Enter target X coordinate: "))
    target_y = float(input("Enter target Y coordinate: "))

    start_theta1 = 20
    start_theta2 = 20
    frames = 100

    target_theta1, target_theta2 = inverse_kinematics(target_x, target_y)

    theta1_path = generate_joint_trajectory(start_theta1, target_theta1, frames)
    theta2_path = generate_joint_trajectory(start_theta2, target_theta2, frames)

    final_points = forward_kinematics(theta1_path[-1], theta2_path[-1])
    final_x, final_y = final_points["end_effector"]

    error_x = final_x - target_x
    error_y = final_y - target_y
    final_error = (error_x**2 + error_y**2) ** 0.5

    print("\nTarget reached calculation complete.")
    print(f"Target position: ({target_x:.3f}, {target_y:.3f})")
    print(f"Final position:  ({final_x:.3f}, {final_y:.3f})")
    print(f"Theta1 target: {target_theta1:.2f}°")
    print(f"Theta2 target: {target_theta2:.2f}°")
    print(f"Final position error: {final_error:.6f} m")


if __name__ == "__main__":
    main()