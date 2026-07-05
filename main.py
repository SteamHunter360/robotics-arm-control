from src.forward_kinematics import forward_kinematics
from src.inverse_kinematics import inverse_kinematics
from src.trajectory_planning import (
    generate_joint_trajectory,
    animate_joint_trajectory,
)
from src.metrics import (
    calculate_position_error,
    calculate_path_length,
    calculate_max_step_distance,
)


def main():
    target_x = float(input("Enter target X coordinate: "))
    target_y = float(input("Enter target Y coordinate: "))

    start_theta1 = 20
    start_theta2 = 20
    frames = 100

    target_theta1, target_theta2 = inverse_kinematics(
        target_x,
        target_y,
    )

    theta1_path = generate_joint_trajectory(
        start_theta1,
        target_theta1,
        frames,
    )

    theta2_path = generate_joint_trajectory(
        start_theta2,
        target_theta2,
        frames,
    )

    end_effector_path = []

    for theta1, theta2 in zip(theta1_path, theta2_path):
        points = forward_kinematics(theta1, theta2)
        end_effector_path.append(points["end_effector"])

    final_x, final_y = end_effector_path[-1]

    final_error = calculate_position_error(
        final_x,
        final_y,
        target_x,
        target_y,
    )

    path_length = calculate_path_length(end_effector_path)

    max_step_distance = calculate_max_step_distance(end_effector_path)

    print("\nTarget reached calculation complete.")
    print(f"Target position: ({target_x:.3f}, {target_y:.3f})")
    print(f"Final position:  ({final_x:.3f}, {final_y:.3f})")
    print(f"Theta1 target: {target_theta1:.2f}°")
    print(f"Theta2 target: {target_theta2:.2f}°")
    print(f"Final position error: {final_error:.6f} m")
    print(f"Total path length: {path_length:.6f} m")
    print(f"Maximum step distance: {max_step_distance:.6f} m")

    animate_joint_trajectory(theta1_path, theta2_path)


if __name__ == "__main__":
    main()