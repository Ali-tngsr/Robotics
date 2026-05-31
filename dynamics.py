"""
PHASE 3–4: ENERGIES & EQUATIONS OF MOTION
Kinetic energy T, Potential energy U, and dynamic model via Euler-Lagrange.

Core: assemble M(θ), C(θ), K, D(θ,φ) and solve for accelerations.
"""
import numpy as np
from params import L, m_b, m_d, I_b, I_xx, E
from taylor_factors import get_all_H, get_all_dH, H1, H2, H3, H4, H5, H6, H7, H8


# ─────────────────────────────────────────────────────────────────────────────
# MASS MATRIX M(θ)  — Eq. (21)
# ─────────────────────────────────────────────────────────────────────────────

def mass_matrix(theta):
    """
    2×2 symmetric inertia matrix.
    
    M11 = ℓ² m_b H1 + ℓ I_b H3 + ℓ² m_d H5 + I_xx H7
    M22 = ℓ² m_b H2 + ℓ I_b H4 + ℓ² m_d H6 + I_xx H8
    M12 = M21 = 0
    """
    h = get_all_H(theta)
    
    M11 = L**2 * m_b * h[0] + L * I_b * h[2] + L**2 * m_d * h[4] + I_xx * h[6]
    M22 = L**2 * m_b * h[1] + L * I_b * h[3] + L**2 * m_d * h[5] + I_xx * h[7]
    
    return np.array([[M11,  0.0],
                     [0.0, M22]])


# ─────────────────────────────────────────────────────────────────────────────
# CORIOLIS / CENTRIPETAL MATRIX C(θ)  — Eq. (22)
# ─────────────────────────────────────────────────────────────────────────────

def coriolis_matrix(theta):
    """
    2×3 matrix C such that Q_coriolis = C · [θ̇², θ̇φ̇, φ̇²]^T.
    
    From Eq. (22):
      C11 = (1/2) dM11/dθ
      C12 = C21 = C23 = 0
      C13 = -(1/2) dM22/dθ
      C22 = dM22/dθ
    """
    dh = get_all_dH(theta)
    
    dM11_dtheta = L**2 * m_b * dh[0] + L * I_b * dh[2] + L**2 * m_d * dh[4] + I_xx * dh[6]
    dM22_dtheta = L**2 * m_b * dh[1] + L * I_b * dh[3] + L**2 * m_d * dh[5] + I_xx * dh[7]
    
    C11 =  0.5 * dM11_dtheta
    C13 = -0.5 * dM22_dtheta
    C22 =        dM22_dtheta
    
    return np.array([[C11,   0.0, C13],
                     [0.0,  C22,  0.0]])


# ─────────────────────────────────────────────────────────────────────────────
# STIFFNESS MATRIX K  — Eq. (23)
# ─────────────────────────────────────────────────────────────────────────────

def stiffness_matrix():
    """
    2×2 elastic stiffness (K22 = 0 because no φ bending).
    
    K11 = E I_b / L
    K22 = 0
    """
    K11 = E * I_b / L
    return np.array([[K11,  0.0],
                     [0.0,  0.0]])


# ─────────────────────────────────────────────────────────────────────────────
# ACTUATION MAP D(θ,φ)  — Eq. (24)
# ─────────────────────────────────────────────────────────────────────────────

def force_matrix(theta, phi, r=None):
    """
    2×2 matrix D mapping cable tensions [F1, F2]^T to generalized forces [Q1, Q2]^T.
    
    Three cables fixed at γ1=0, γ2=2π/3, γ3=4π/3 from Eq. (19).
    We control cables 1 and 2.
    
    D11 = r cos(γ1 - φ) = r cos(φ)
    D12 = r cos(γ2 - φ) = r cos(2π/3 - φ)
    D21 = -r θ sin(γ1 - φ) = -r θ sin(φ)
    D22 =  r θ sin(γ2 - φ) = r θ sin(2π/3 - φ)
    """
    from params import r_cab
    if r is None:
        r = r_cab
    
    gam1 = 0.0
    gam2 = 2*np.pi/3
    
    D11 =  r * np.cos(gam1 - phi)
    D12 =  r * np.cos(gam2 - phi)
    D21 = -r * theta * np.sin(gam1 - phi)
    D22 =  r * theta * np.sin(gam2 - phi)
    
    return np.array([[D11, D12],
                     [D21, D22]])

# ─────────────────────────────────────────────────────────────────────────────
# DAMPING MATRIX B (Added for physical realism and to match Fig. 8)
# ─────────────────────────────────────────────────────────────────────────────

def damping_matrix():
    """
    2x2 viscous damping matrix.
    Since Eq 20 in the paper lacks a damping term, the theoretical system is conservative.
    To replicate the ~37.68s decay seen in Fig. 8, a small structural damping is required.
    """
    B11 = 0.002  # Tuned to achieve ~37s settling time
    B22 = 0.002
    return np.array([[B11,  0.0],
                     [0.0,  B22]])
# ─────────────────────────────────────────────────────────────────────────────
# FULL EQUATIONS OF MOTION  — Eq. (20)
# ─────────────────────────────────────────────────────────────────────────────

def state_derivative(t, state, force_func):
    """
    ODE function for scipy.integrate.solve_ivp.
    
    state = [θ, φ, θ̇, φ̇]
    
    Returns d/dt[θ, φ, θ̇, φ̇] = [θ̇, φ̇, θ̈, φ̈]
    
    Equations (from Eq. 20):
        M·q̈ = D·F - C·v - K·q
    where v = [θ̇², θ̇φ̇, φ̇²]^T
    """
    theta, phi, theta_dot, phi_dot = state
    
    # Protect singularity
    if abs(theta) < 1e-8:
        theta = 1e-8
    
    # Get forces from controller
    F = force_func(t)  # [F1, F2]
    
    # Assemble matrices
    M = mass_matrix(theta)
    C = coriolis_matrix(theta)
    K = stiffness_matrix()
    D = force_matrix(theta, phi)
    
    q = np.array([theta, phi])
    vel = np.array([theta_dot**2, theta_dot*phi_dot, phi_dot**2])
    
    # Right-hand side: M·q̈ = D·F - C·v - K·q
    rhs = D @ F - C @ vel - K @ q
    
    # Solve for accelerations (M is always invertible)
    try:
        q_ddot = np.linalg.solve(M, rhs)
    except np.linalg.LinAlgError:
        # Fallback (should never happen)
        q_ddot = np.linalg.lstsq(M, rhs, rcond=None)[0]
    
    return [theta_dot, phi_dot, q_ddot[0], q_ddot[1]]


# ─────────────────────────────────────────────────────────────────────────────
# INVERSE DYNAMICS  — solve for F given trajectory
# ─────────────────────────────────────────────────────────────────────────────

def inverse_dynamics(theta, phi, theta_dot, phi_dot, theta_ddot, phi_ddot):
    """
    Solve Eq. (20) for cable forces [F1, F2] given desired state trajectory.
    
    D·F = M·q̈ + C·v + K·q
    """
    M = mass_matrix(theta)
    C = coriolis_matrix(theta)
    K = stiffness_matrix()
    D = force_matrix(theta, phi)
    
    q = np.array([theta, phi])
    q_ddot = np.array([theta_ddot, phi_ddot])
    vel = np.array([theta_dot**2, theta_dot*phi_dot, phi_dot**2])
    
    rhs = M @ q_ddot + C @ vel + K @ q
    
    # Solve for F (use pinv for robustness)
    if abs(np.linalg.det(D)) < 1e-10:
        F = np.linalg.lstsq(D, rhs, rcond=None)[0]
    else:
        F = np.linalg.solve(D, rhs)
    
    return F  # [F1, F2]


# ─────────────────────────────────────────────────────────────────────────────
# KINETIC & POTENTIAL ENERGY (for validation/analysis)
# ─────────────────────────────────────────────────────────────────────────────

def total_kinetic_energy(theta, theta_dot, phi_dot):
    """
    Total kinetic energy T = T_backbone + T_disks (translational + rotational).
    
    From Eqs. (9), (14), (15), (16).
    """
    h = get_all_H(theta)
    
    # Backbone translational
    Tb_trans = 0.5 * L**2 * m_b * (h[0] * theta_dot**2 + h[1] * phi_dot**2)
    
    # Backbone rotational
    Tb_rot = 0.5 * L * I_b * (h[2] * theta_dot**2 + h[3] * phi_dot**2)
    
    # Disk translational
    Td_trans = 0.5 * L**2 * m_d * (h[4] * theta_dot**2 + h[5] * phi_dot**2)
    
    # Disk rotational
    Td_rot = 0.5 * I_xx * (h[6] * theta_dot**2 + h[7] * phi_dot**2)
    
    return Tb_trans + Tb_rot + Td_trans + Td_rot


def total_potential_energy(theta):
    """
    Total potential energy U = elastic energy (gravity negligible, Eq. 18).
    
    U = (E I_b / 2L) θ²
    """
    return (E * I_b / (2 * L)) * theta**2


def lagrangian(theta, theta_dot, phi_dot):
    """L = T - U"""
    T = total_kinetic_energy(theta, theta_dot, phi_dot)
    U = total_potential_energy(theta)
    return T - U
