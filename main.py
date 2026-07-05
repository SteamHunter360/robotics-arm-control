from src.inverse_kinematics import inverse_kinematics
from src.trajectory_planning import generate_joint_trajectory
from src.two_joint_trajectory_tracking import simulate_two_joint_trajectory_tracking
from src.cartesian_analysis import (
    calculate_end_effector_path,
    analyse_cartesian_tracking,
)
from src.controlled_robot_visualisation import animate_desired_vs_actual_paths


def main():
    target_x = float(input("Enter target X coordinate: "))
    target_y = float(input("Enter target Y coordinate: "))

    start_theta1 = 20.0
    start_theta2 = 20.0
    frames = 100

    target_theta1, target_theta2 = inverse_kinematics(
        target_x,
        target_y,
    )

    desired_theta1 = generate_joint_trajectory(
        start_theta1,
        target_theta1,
        frames,
    )

    desired_theta2 = generate_joint_trajectory(
        start_theta2,
        target_theta2,
        frames,
    )

    tracking_results = simulate_two_joint_trajectory_tracking(
        desired_theta1,
        desired_theta2,
        initial_theta1=start_theta1,
        initial_theta2=start_theta2,
    )

    desired_path = calculate_end_effector_path(
        desired_theta1,
        desired_theta2,
    )

    actual_path = calculate_end_effector_path(
        tracking_results["actual_theta1_values"],
        tracking_results["actual_theta2_values"],
    )

    cartesian_results = analyse_cartesian_tracking(
        desired_path,
        actual_path,
    )

    print("\nControlled Robot Arm Results")
    print(f"Target position: ({target_x:.3f}, {target_y:.3f})")
    print(
        f"Target joint angles: "
        f"θ1={target_theta1:.2f}°, θ2={target_theta2:.2f}°"
    )
    print(f"Final Cartesian error: {cartesian_results['final_error']:.6f} m")
    print(f"Maximum Cartesian error: {cartesian_results['max_error']:.6f} m")
    print(f"RMS Cartesian error: {cartesian_results['rms_error']:.6f} m")
    print(f"Desired path length: {cartesian_results['desired_path_length']:.6f} m")
    print(f"Actual path length: {cartesian_results['actual_path_length']:.6f} m")
    print(f"Desired max step: {cartesian_results['desired_max_step']:.6f} m")
    print(f"Actual max step: {cartesian_results['actual_max_step']:.6f} m")

    animate_desired_vs_actual_paths(
        desired_path,
        actual_path,
    )


if __name__ == "__main__":
    main()