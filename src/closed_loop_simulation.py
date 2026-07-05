from src.pid_controller import PIDController
from src.joint_simulation import JointSimulation


def simulate_joint_position_control(
    target_angle,
    initial_angle=0.0,
    duration=2.0,
    dt=0.01,
    kp=30.0,
    ki=0.0,
    kd=5.0,
):
    controller = PIDController(
        kp=kp,
        ki=ki,
        kd=kd,
        dt=dt,
        output_limit=100.0,
    )

    joint = JointSimulation(
        initial_angle=initial_angle,
        initial_velocity=0.0,
        inertia=1.0,
        damping=2.0,
        dt=dt,
    )

    time_values = []
    angle_values = []
    control_values = []

    steps = int(duration / dt)

    for step in range(steps):
        time = step * dt

        control_input = controller.update(
            setpoint=target_angle,
            measurement=joint.angle,
        )

        angle = joint.update(control_input)

        time_values.append(time)
        angle_values.append(angle)
        control_values.append(control_input)

    return time_values, angle_values, control_values