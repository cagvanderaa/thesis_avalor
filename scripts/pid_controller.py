class PIDController:
    def __init__(self, Kp, Ki, Kd):
        """
        Initializes the PID controller with specified gains.

        Args:
            Kp (float): Proportional gain.
            Ki (float): Integral gain.
            Kd (float): Derivative gain.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.previous_error = 0.0
        self.integral_limit = None  # Optional: Limit for integral term to prevent windup

    def update(self, error, dt):
        """
        Updates the PID controller with the given error and time step.

        Args:
            error (float): The current error value.
            dt (float): Time step in seconds.

        Returns:
            float: The control output.
        """
        if dt <= 0.0:
            dt = 1e-6  # Prevent division by zero and negative time steps

        # Update integral term with optional clamping to prevent integral windup
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)

        # Calculate derivative term
        derivative = (error - self.previous_error) / dt

        # Compute PID output
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Update previous error
        self.previous_error = error

        return output

    def update_gains(self, Kp, Ki, Kd):
        """
        Updates the PID controller gains.

        Args:
            Kp (float): New proportional gain.
            Ki (float): New integral gain.
            Kd (float): New derivative gain.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
  

    def reset(self):
        """
        Resets the integral and derivative terms of the PID controller.
        """
        self.integral = 0.0
        self.previous_error = 0.0

