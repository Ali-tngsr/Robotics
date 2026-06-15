"""
PID Controller for 1-DOF/2-DOF bending angle tracking.
Extracted for better modularity and Separation of Concerns.
"""

class SimplePIDController:
    """Simple 1-DOF PID for bending angle θ."""
    
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_error = 0.0
        self.prev_error = 0.0
    
    def update(self, error, dt):
        """Compute control output for given error and time step."""
        self.integral_error += error * dt
        derivative_error = (error - self.prev_error) / dt if dt > 0 else 0.0
        
        output = self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative_error
        self.prev_error = error
        
        return output
