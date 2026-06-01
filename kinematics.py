"""
PHASE 1: KINEMATICS
Constant-curvature model with zero torsion.

Position vector (Eq. 1), Orientation matrix (Eq. 2),
Linear & Angular velocities (Eqs. 3-4).
"""
import numpy as np


def position(s, theta, phi, L):
    """
    Eq. (1): Position vector r_s at arc-length s along the backbone.
    
    r_s = [
        (s/θ)(1 - cos θ_s) cos φ,
        (s/θ)(1 - cos θ_s) sin φ,
        (s/θ) sin θ_s
    ]^T
    
    where θ_s = (s/L)·θ is the local bending angle.
    
    Parameters
    ----------
    s     : float or array  arc-length in [0, L]
    theta : float           bending angle (rad)
    phi   : float           orientation angle (rad)
    L     : float           total backbone length
    
    Returns
    -------
    r : ndarray shape (3,) or (3, n)
    """
    # Avoid singularity at theta → 0
    if np.abs(theta) < 1e-10:
        theta = 1e-10
    
    theta_s = (s / L) * theta
    ratio = 1.0 / theta  # Factor (s/θ) in limit becomes ell/theta
    
    x = ratio * (1 - np.cos(theta_s)) * np.cos(phi)
    y = ratio * (1 - np.cos(theta_s)) * np.sin(phi)
    z = ratio * np.sin(theta_s)
    
    return np.array([x, y, z])


def orientation_matrix(s, theta, phi, L):
    """
    Eq. (2): Orientation matrix R_s via three sequential rotations.
    
    R_s = Rot(Z, φ) · Rot(Y, θ_s) · Rot(Z, -φ)
    
    Columns are: [n_s, b_s, t_s] (normal, binormal, tangent vectors).
    """
    theta_s = (s / L) * theta
    
    # Rot(Z, φ)
    c_phi, s_phi = np.cos(phi), np.sin(phi)
    R_z_phi = np.array([[c_phi, -s_phi, 0],
                        [s_phi,  c_phi, 0],
                        [    0,      0, 1]])
    
    # Rot(Y, θ_s)
    c_ts, s_ts = np.cos(theta_s), np.sin(theta_s)
    R_y_ts = np.array([[ c_ts, 0, s_ts],
                       [    0, 1,    0],
                       [-s_ts, 0, c_ts]])
    
    # Rot(Z, -φ)
    R_z_negphi = np.array([[ c_phi, s_phi, 0],
                           [-s_phi, c_phi, 0],
                           [     0,     0, 1]])
    
    return R_z_phi @ R_y_ts @ R_z_negphi


def tangent_vector(s, theta, phi, L):
    """
    Eq. (4): Tangent vector t_s (third column of R_s).
    
    t_s = [cos(φ)·sin(θ_s), sin(φ)·sin(θ_s), cos(θ_s)]^T
    """
    theta_s = (s / L) * theta
    return np.array([np.cos(phi) * np.sin(theta_s),
                     np.sin(phi) * np.sin(theta_s),
                     np.cos(theta_s)])


def linear_velocity(s, theta, phi, theta_dot, phi_dot, L):
    """
    Linear velocity v_s = d(r_s)/dt (chain rule through θ, φ).
    
    v_s = ∂r_s/∂θ · θ̇ + ∂r_s/∂φ · φ̇
    """
    eps = 1e-10
    if np.abs(theta) < eps:
        theta = eps
    
    theta_s = (s / L) * theta
    
    # ∂r_s/∂θ  (note: ∂θ_s/∂θ = s/L)
    dr_dtheta = np.array([
        (1/theta) * np.sin(theta_s) * (s/L) * np.cos(phi) 
        - (1/theta**2) * (1 - np.cos(theta_s)) * np.cos(phi),
        (1/theta) * np.sin(theta_s) * (s/L) * np.sin(phi)
        - (1/theta**2) * (1 - np.cos(theta_s)) * np.sin(phi),
        (1/theta) * np.cos(theta_s) * (s/L) - (1/theta**2) * np.sin(theta_s)
    ])
    
    # ∂r_s/∂φ
    dr_dphi = np.array([
        -(1/theta) * (1 - np.cos(theta_s)) * np.sin(phi),
         (1/theta) * (1 - np.cos(theta_s)) * np.cos(phi),
        0.0
    ])
    
    return dr_dtheta * theta_dot + dr_dphi * phi_dot


def _skew(v):
    """Skew-symmetric matrix of vector v."""
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def angular_velocity(s, theta, phi, theta_dot, phi_dot, L):
    """
    Eq. (3): Angular velocity ω_s = [t̂_s]·ṫ_s
    
    where t̂_s is the skew matrix of tangent vector t_s.
    """
    theta_s = (s / L) * theta
    t_s = tangent_vector(s, theta, phi, L)
    
    # dt_s/dθ
    dt_dtheta = np.array([
        np.cos(phi) * np.cos(theta_s) * (s / L),
        np.sin(phi) * np.cos(theta_s) * (s / L),
        -np.sin(theta_s) * (s / L)
    ])
    
    # dt_s/dφ
    dt_dphi = np.array([
        -np.sin(phi) * np.sin(theta_s),
         np.cos(phi) * np.sin(theta_s),
        0.0
    ])
    
    t_dot = dt_dtheta * theta_dot + dt_dphi * phi_dot
    T_hat = _skew(t_s)
    
    return T_hat @ t_dot
    
def jacobian_end_effector(theta, phi, L):
    """
    Computes the 3x2 Jacobian matrix for the end-effector (s=L).
    v_e = J_e * [theta_dot, phi_dot]^T
    """
    eps = 1e-10
    if np.abs(theta) < eps:
        theta = eps
    
    # مشتقات موقعیت نقطه انتهایی نسبت به تتا (s = L)
    dr_dtheta = np.array([
        (L/theta) * np.sin(theta) * np.cos(phi) - (L/theta**2) * (1 - np.cos(theta)) * np.cos(phi),
        (L/theta) * np.sin(theta) * np.sin(phi) - (L/theta**2) * (1 - np.cos(theta)) * np.sin(phi),
        (L/theta) * np.cos(theta) - (L/theta**2) * np.sin(theta)
    ])
    
    # مشتقات موقعیت نقطه انتهایی نسبت به فی (s = L)
    dr_dphi = np.array([
        -(L/theta) * (1 - np.cos(theta)) * np.sin(phi),
         (L/theta) * (1 - np.cos(theta)) * np.cos(phi),
        0.0
    ])
    
    return np.column_stack((dr_dtheta, dr_dphi))
