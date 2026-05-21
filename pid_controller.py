import numpy as np
from dynamics import assemble_matrices_with_payload

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        """
        Kp, Ki, Kd: Diagonal 2x2 matrices for proportional, integral, derivative gains
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_error = np.zeros(2)
        self.prev_error = np.zeros(2)
        
    def compute_control_law(self, state, desired_state, dt):
        theta, phi = state[0], state[1]
        theta_d, phi_d = desired_state[0], desired_state[1]
        
        # Protect against singularity
        if abs(theta) < 1e-6:
            theta = 1e-6
            
        q = np.array([theta, phi])
        q_d = np.array([theta_d, phi_d])
        
        # 1. Calculate Errors
        e = q_d - q
        self.integral_error += e * dt
        de = (e - self.prev_error) / dt
        
        # 2. Compute Generalized Force (Tau)
        tau = (self.Kp @ e) + (self.Ki @ self.integral_error) + (self.Kd @ de)
        
        # 3. Map to Cable Tensions F = D^-1 * Tau
        # The classical PID relies strictly on nominal kinematics (0g payload assumption)
        _, _, _, D, _ = assemble_matrices_with_payload(theta, phi, m_p=0.0)
        
        F_req = np.linalg.pinv(D) @ tau
        F_req[F_req < 0] = 0.0  # Cables can only pull
        
        self.prev_error = e
        return F_req, e