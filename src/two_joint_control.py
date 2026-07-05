from src.pid_controller import PIDController
from src.joint_simulation import JointSimulation


def simulate_two_joint_position_control(
    target_theta1,
    target_theta2,
    initial_theta1=20.0,
    initial_theta2=20.0,
    duration=3.0,
    dt=0.01,
    kp=30.0,
    ki=0.0,
    kd=5.0,
):
    controller_1 = PIDController(kp, ki, kd, dt, output_limit=100.0)
    controller_2 = PIDController(kp, ki, kd, dt, output_limit=100.0)

    joint_1 = JointSimulation(
        initial_angle=initial_theta1,
        initial_velocity=0.0,
        inertia=1.0,
        damping=2.0,
        dt=dt,
    )

    joint_2 = JointSimulation(
        initial_angle=initial_theta2,
        initial_velocity=0.0,
        inertia=1.0,
        damping=2.0,
        dt=dt,
    )

    time_values = []
    theta1_values = []
    theta2_values = []
    control_1_values = []
    control_2_values = []

    steps = int(duration / dt)

    for step in range(steps):
        time = step * dt

        control_1 = controller_1.update(target_theta1, joint_1.angle)
        control_2 = controller_2.update(target_theta2, joint_2.angle)

        theta1 = joint_1.update(control_1)
        theta2 = joint_2.update(control_2)

        time_values.append(time)
        theta1_values.append(theta1)
        theta2_values.append(theta2)
        control_1_values.append(control_1)
        control_2_values.append(control_2)

    return {
        "time_values": time_values,
        "theta1_values": theta1_values,
        "theta2_values": theta2_values,
        "control_1_values": control_1_values,
        "control_2_values": control_2_values,
    }