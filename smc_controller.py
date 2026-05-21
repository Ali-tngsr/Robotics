import numpy as np
from dynamics import assemble_matrices_with_payload

class SlidingModeController:
    def __init__(self, Lambda, K_sw, m_hat, phi_bound):
        """
        Lambda: Diagonal matrix for sliding surface slope (np.array 2x2)
        K_sw: Diagonal matrix for switching gains (np.array 2x2)
        m_hat: The *estimated* payload mass the controller assumes is attached
        phi_bound: Boundary layer thickness to prevent chattering (np.array 1x2 or scalar)
        """
        self.Lambda = Lambda
        self.K_sw = K_sw
        self.m_hat = m_hat
        self.phi_bound = phi_bound
        
    def compute_control_law(self, state, desired_state):
        theta, phi, dtheta, dphi = state
        theta_d, phi_d, dtheta_d, dphi_d, ddtheta_d, ddphi_d = desired_state
        
        # Protect against singularity
        if abs(theta) < 1e-6:
            theta = 1e-6
            
        q = np.array([theta, phi])
        dq = np.array([dtheta, dphi])
        q_d = np.array([theta_d, phi_d])
        dq_d = np.array([dtheta_d, dphi_d])
        ddq_d = np.array([ddtheta_d, ddphi_d])
        
        # 1. Calculate Errors
        e = q_d - q
        de = dq_d - dq
        
        # 2. Calculate Sliding Surface S
        S = de + self.Lambda @ e
        
        # 3. Assemble matrices using our ESTIMATED payload mass
        M, C, K, D, G = assemble_matrices_with_payload(theta, phi, self.m_hat)
        v_vec = np.array([dtheta**2, dphi**2, dtheta*dphi])
        
        # 4. Equivalent Control (Q_eq)
        Q_eq = M @ (ddq_d + self.Lambda @ de) + (C @ v_vec) + (K @ q) + G
        
        # 5. Switching Control (Q_sw) using Saturation instead of Sign
        # S_sat = np.clip(S / phi_bound, -1.0, 1.0)
        S_sat = np.clip(S / self.phi_bound, -1.0, 1.0)
        Q_sw = self.K_sw @ S_sat
        
        # 6. Total Generalized Force
        Q_total = Q_eq + Q_sw
        
        # 7. Map to Cable Tensions F = D^-1 * Q_total
        F_req = np.linalg.pinv(D) @ Q_total
        
        # Ensure cables can only PULL (Tension >= 0)
        F_req[F_req < 0] = 0.0
        
        return F_req, S, e