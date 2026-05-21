import numpy as np

def position_vector(s, theta, phi, L):
    """
    Eq. (1): Position vector r_s at arc length s [cite: 108]
    """
    # Handle the singularity at theta = 0 safely
    if np.abs(theta) < 1e-6:
        theta_s = (s / L) * 1e-6
    else:
        theta_s = (s / L) * theta
        
    x = (s / theta_s) * (1 - np.cos(theta_s)) * np.cos(phi)
    y = (s / theta_s) * (1 - np.cos(theta_s)) * np.sin(phi)
    z = (s / theta_s) * np.sin(theta_s)
    
    return np.array([x, y, z])

def orientation_matrix(s, theta, phi, L):
    """
    Eq. (2): Orientation matrix R_s via sequential rotations [cite: 111]
    """
    theta_s = (s / L) * theta
    
    # rot(Z0, phi)
    R_z_phi = np.array([
        [np.cos(phi), -np.sin(phi), 0],
        [np.sin(phi), np.cos(phi), 0],
        [0, 0, 1]
    ])
    
    # rot(Y0, theta_s)
    R_y_theta = np.array([
        [np.cos(theta_s), 0, np.sin(theta_s)],
        [0, 1, 0],
        [-np.sin(theta_s), 0, np.cos(theta_s)]
    ])
    
    # rot(Z0, -phi)
    R_z_neg_phi = np.array([
        [np.cos(-phi), -np.sin(-phi), 0],
        [np.sin(-phi), np.cos(-phi), 0],
        [0, 0, 1]
    ])
    
    return R_z_phi @ R_y_theta @ R_z_neg_phi