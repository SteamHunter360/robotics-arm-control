from src.pid_controller import PIDController
from src.joint_simulation import JointSimulation


def simulate_two_joint_trajectory_tracking(
    desired_theta1_values,
    desired_theta2_values,
    initial_theta1=20.0,
    initial_theta2=20.0,
    dt=0.01,
    kp=30.0,
    ki=0.0,
    kd=5.0,
):
    if len(desired_theta1_values) != len(desired_theta2_values):
        raise ValueError("Desired joint trajectories must have the same length.")

    controller_1 = PIDController(kp, ki, kd, dt, output_limit=100.0)
    controller_2 = PIDController(kp, ki, kd, dt, output_limit=100.0)

    joint_1 = JointSimulation(initial_theta1, 0.0, 1.0, 2.0, dt)
    joint_2 = JointSimulation(initial_theta2, 0.0, 1.0, 2.0, dt)

    time_values = []
    actual_theta1_values = []
    actual_theta2_values = []
    control_1_values = []
    control_2_values = []

    for step, (desired_theta1, desired_theta2) in enumerate(
        zip(desired_theta1_values, desired_theta2_values)
    ):
        time = step * dt

        control_1 = controller_1.update(desired_theta1, joint_1.angle)
        control_2 = controller_2.update(desired_theta2, joint_2.angle)

        actual_theta1 = joint_1.update(control_1)
        actual_theta2 = joint_2.update(control_2)

        time_values.append(time)
        actual_theta1_values.append(actual_theta1)
        actual_theta2_values.append(actual_theta2)
        control_1_values.append(control_1)
        control_2_values.append(control_2)

    return {
        "time_values": time_values,
        "desired_theta1_values": desired_theta1_values,
        "desired_theta2_values": desired_theta2_values,
        "actual_theta1_values": actual_theta1_values,
        "actual_theta2_values": actual_theta2_values,
        "control_1_values": control_1_values,
        "control_2_values": control_2_values,
    }