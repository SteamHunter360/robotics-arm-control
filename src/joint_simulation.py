class JointSimulation:
    """
    Simple first-order rotational joint model.

    The control input produces angular acceleration.
    Damping resists joint motion.
    """

    def __init__(
        self,
        initial_angle=0.0,
        initial_velocity=0.0,
        inertia=1.0,
        damping=1.0,
        dt=0.01,
    ):
        if inertia <= 0:
            raise ValueError("inertia must be greater than zero.")

        if dt <= 0:
            raise ValueError("dt must be greater than zero.")

        self.angle = initial_angle
        self.velocity = initial_velocity

        self.inertia = inertia
        self.damping = damping
        self.dt = dt

    def update(self, control_input):
        """
        Advance the joint simulation by one time step.

        control_input is treated as applied joint torque.

        Returns:
            Updated joint angle.
        """

        acceleration = (
            control_input - self.damping * self.velocity
        ) / self.inertia

        self.velocity += acceleration * self.dt
        self.angle += self.velocity * self.dt

        return self.angle

    def reset(self, angle=0.0, velocity=0.0):
        self.angle = angle
        self.velocity = velocity