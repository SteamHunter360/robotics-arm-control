class PIDController:
    """
    Basic PID controller for joint position control.
    """

    def __init__(self, kp, ki, kd, dt, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_limit = output_limit

        self.integral = 0.0
        self.previous_error = 0.0

    def update(self, setpoint, measurement):
        error = setpoint - measurement

        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        self.previous_error = error

        if self.output_limit is not None:
            output = max(
                -self.output_limit,
                min(self.output_limit, output),
            )

        return output

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0