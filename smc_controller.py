"""
Sliding Mode Controller (SMC) for 1-DOF bending angle tracking.
Includes a saturation function to eliminate chattering.
"""
import numpy as np

class SMCController:
    def __init__(self, lambda_c=8.0, K_s=15.0, epsilon=0.05):
        self.lambda_c = lambda_c
        self.K_s = K_s
        self.epsilon = epsilon  # Boundary layer to prevent chattering (Day 10 fix)

    def update(self, error, error_dot):
        """Compute control output using Sliding Mode logic."""
        # Sliding surface: s = e_dot + lambda * e
        s = error_dot + self.lambda_c * error
        
        # Saturation function instead of pure sign() to prevent chattering
        if abs(s) <= self.epsilon:
            sat = s / self.epsilon
        else:
            sat = np.sign(s)
            
        # Control law: u = K * sat(s)
        output = self.K_s * sat
        return output
