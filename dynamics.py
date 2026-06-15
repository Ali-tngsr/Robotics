"""
PHASE 3–4: ENERGIES & EQUATIONS OF MOTION
Kinetic energy T, Potential energy U, and dynamic model via Euler-Lagrange.

Core: assemble M(θ), C(θ), K, D(θ,φ) and solve for accelerations.
(Refactored: Dependency Injection for payload & Centralized Imports)
"""
import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# IMPORTS 
# ─────────────────────────────────────────────────────────────────────────────
from taylor_factors import get_all_H, get_all_dH
from kinematics import jacobian_end_effector

# وارد کردن پارامترهای ثابت ربات (پایه)
# توجه: m_p و g دیگر مستقیماً از params خوانده نمی‌شوند تا بتوان در شبیه‌سازی آن‌ها را به صورت متغیر تزریق کرد.
from params import L, m_b, m_d, I_b, I_xx, E, r_cab


# ─────────────────────────────────────────────────────────────────────────────
# MASS MATRIX M(θ)  — Eq. (21)
# ─────────────────────────────────────────────────────────────────────────────
def mass_matrix(theta, m_p=0.0):
    """
    محاسبه ماتریس جرم با قابلیت دریافت جرم بار اضافی (m_p) به عنوان ورودی.
    """
    h = get_all_H(theta)
    
    M11_base = L**2 * m_b * h[0] + L * I_b * h[2] + L**2 * m_d * h[4] + I_xx * h[6]
    M22_base = L**2 * m_b * h[1] + L * I_b * h[3] + L**2 * m_d * h[5] + I_xx * h[7]
    M_base = np.array([[M11_base,  0.0],
                       [0.0, M22_base]])
    
    # اثر جرم بار اضافی
    J_e = jacobian_end_effector(theta, 0.0, L)
    M_payload = m_p * (J_e.T @ J_e)
    
    return M_base + M_payload


# ─────────────────────────────────────────────────────────────────────────────
# CORIOLIS / CENTRIPETAL MATRIX C(θ)  — Eq. (22)
# ─────────────────────────────────────────────────────────────────────────────
def coriolis_matrix(theta, m_p=0.0):
    dh = get_all_dH(theta)
    
    dM11_base = L**2 * m_b * dh[0] + L * I_b * dh[2] + L**2 * m_d * dh[4] + I_xx * dh[6]
    dM22_base = L**2 * m_b * dh[1] + L * I_b * dh[3] + L**2 * m_d * dh[5] + I_xx * dh[7]
    
    delta = 1e-5
    J_plus = jacobian_end_effector(theta + delta, 0.0, L)
    M_p_plus = m_p * (J_plus.T @ J_plus)
    
    J_minus = jacobian_end_effector(theta - delta, 0.0, L)
    M_p_minus = m_p * (J_minus.T @ J_minus)
    
    dM_p_dtheta = (M_p_plus - M_p_minus) / (2 * delta)
    
    dM11_total = dM11_base + dM_p_dtheta[0, 0]
    dM22_total = dM22_base + dM_p_dtheta[1, 1]
    
    C11 =  0.5 * dM11_total
    C13 = -0.5 * dM22_total
    C22 =        dM22_total
    
    return np.array([[C11,   0.0, C13],
                     [0.0,  C22,  0.0]])


# ─────────────────────────────────────────────────────────────────────────────
# STIFFNESS MATRIX K  — Eq. (23)
# ─────────────────────────────────────────────────────────────────────────────
def stiffness_matrix():
    """
    2×2 elastic stiffness (K22 = 0 because no φ bending).
    """
    K11 = E * I_b / L
    return np.array([[K11,  0.0],
                     [0.0,  0.0]])


# ─────────────────────────────────────────────────────────────────────────────
# DAMPING MATRIX B 
# ─────────────────────────────────────────────────────────────────────────────
def damping_matrix():
    """
    2x2 viscous damping matrix to replicate the settling time in the paper.
    """
    B11 = 0.002
    B22 = 0.002
    return np.array([[B11,  0.0],
                     [0.0,  B22]])


# ─────────────────────────────────────────────────────────────────────────────
# GRAVITY VECTOR G(θ)
# ─────────────────────────────────────────────────────────────────────────────
def gravity_vector(theta, m_p=0.0, g=9.81):
    eps = 1e-10
    if np.abs(theta) < eps:
        dUg_dtheta = 0.0
    else:
        dUg_dtheta = m_p * g * L * (theta * np.cos(theta) - np.sin(theta)) / (theta**2)
        
    return np.array([dUg_dtheta, 0.0])


# ─────────────────────────────────────────────────────────────────────────────
# ACTUATION MAP D(θ,φ)  — Eq. (24)
# ─────────────────────────────────────────────────────────────────────────────
def force_matrix(theta, phi, r=None):
    if r is None:
        r = r_cab
    
    gam1 = 0.0
    gam2 = 2*np.pi/3
    
    D11 =  r * np.cos(gam1 - phi)
    D12 =  r * np.cos(gam2 - phi)
    D21 =  r * theta * np.sin(gam1 - phi)
    D22 =  r * theta * np.sin(gam2 - phi)
    
    return np.array([[D11, D12],
                     [D21, D22]])


# ─────────────────────────────────────────────────────────────────────────────
# FULL EQUATIONS OF MOTION  — Eq. (20)
# ─────────────────────────────────────────────────────────────────────────────
def state_derivative(t, state, force_func, m_p=0.0, g=9.81):
    theta, phi, theta_dot, phi_dot = state
    
    if abs(theta) < 1e-8:
        theta = 1e-8
    
    F = force_func(t)
    
    M = mass_matrix(theta, m_p)
    C = coriolis_matrix(theta, m_p)
    K = stiffness_matrix()
    D = force_matrix(theta, phi)
    B = damping_matrix()
    G_vec = gravity_vector(theta, m_p, g)
    
    q = np.array([theta, phi])
    q_dot = np.array([theta_dot, phi_dot])
    vel = np.array([theta_dot**2, theta_dot*phi_dot, phi_dot**2])
    
    # بردار گرانش از نیروها کم می‌شود
    rhs = D @ F - C @ vel - K @ q - B @ q_dot - G_vec 
    
    try:
        q_ddot = np.linalg.solve(M, rhs)
    except np.linalg.LinAlgError:
        q_ddot = np.linalg.lstsq(M, rhs, rcond=None)[0]
    
    return [theta_dot, phi_dot, q_ddot[0], q_ddot[1]]


# ─────────────────────────────────────────────────────────────────────────────
# INVERSE DYNAMICS (2 & 3 Cables)
# ─────────────────────────────────────────────────────────────────────────────
def inverse_dynamics(theta, phi, theta_dot, phi_dot, theta_ddot, phi_ddot, m_p=0.0):
    M = mass_matrix(theta, m_p)
    C = coriolis_matrix(theta, m_p)
    K = stiffness_matrix()
    D = force_matrix(theta, phi)
    B = damping_matrix()
    
    q = np.array([theta, phi])
    q_dot = np.array([theta_dot, phi_dot])
    q_ddot = np.array([theta_ddot, phi_ddot])
    vel = np.array([theta_dot**2, theta_dot*phi_dot, phi_dot**2])
    
    rhs = M @ q_ddot + C @ vel + K @ q + B @ q_dot
    
    if abs(np.linalg.det(D)) < 1e-10:
        F = np.linalg.lstsq(D, rhs, rcond=None)[0]
    else:
        F = np.linalg.solve(D, rhs)
    
    return F


def inverse_dynamics_3cables(theta, phi, theta_dot, phi_dot, theta_ddot, phi_ddot, m_p=0.0):
    M = mass_matrix(theta, m_p)
    C = coriolis_matrix(theta, m_p)
    K = stiffness_matrix()
    B = damping_matrix()
    
    q = np.array([theta, phi])
    q_dot = np.array([theta_dot, phi_dot])
    q_ddot = np.array([theta_ddot, phi_ddot])
    vel = np.array([theta_dot**2, theta_dot*phi_dot, phi_dot**2])
    
    rhs = M @ q_ddot + C @ vel + K @ q + B @ q_dot  
    
    gam1, gam2, gam3 = 0.0, 2*np.pi/3, 4*np.pi/3
    
    D3 = np.array([
        [r_cab * np.cos(gam1 - phi), r_cab * np.cos(gam2 - phi), r_cab * np.cos(gam3 - phi)],
        [r_cab * theta * np.sin(gam1 - phi), r_cab * theta * np.sin(gam2 - phi), r_cab * theta * np.sin(gam3 - phi)]
    ])
    
    F_base = np.linalg.pinv(D3) @ rhs
    
    min_F = np.min(F_base)
    if min_F < 0:
        F = F_base - min_F + 0.1
    else:
        F = F_base
        
    return F


# ─────────────────────────────────────────────────────────────────────────────
# KINETIC & POTENTIAL ENERGY (for validation/analysis)
# ─────────────────────────────────────────────────────────────────────────────
def total_kinetic_energy(theta, theta_dot, phi_dot):
    h = get_all_H(theta)
    Tb_trans = 0.5 * L**2 * m_b * (h[0] * theta_dot**2 + h[1] * phi_dot**2)
    Tb_rot = 0.5 * L * I_b * (h[2] * theta_dot**2 + h[3] * phi_dot**2)
    Td_trans = 0.5 * L**2 * m_d * (h[4] * theta_dot**2 + h[5] * phi_dot**2)
    Td_rot = 0.5 * I_xx * (h[6] * theta_dot**2 + h[7] * phi_dot**2)
    return Tb_trans + Tb_rot + Td_trans + Td_rot


def total_potential_energy(theta):
    return (E * I_b / (2 * L)) * theta**2


def lagrangian(theta, theta_dot, phi_dot):
    T = total_kinetic_energy(theta, theta_dot, phi_dot)
    U = total_potential_energy(theta)
    return T - U
