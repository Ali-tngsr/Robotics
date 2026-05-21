
import numpy as np
import params as p
from taylor_factors import get_H_factors, get_dH_factors
from payload_extension import get_payload_factors

def assemble_matrices(theta, phi):
    """
    Assembles M, C_coeffs, K, and D matrices for the Euler-Lagrange equations.
    """
    H = get_H_factors(theta)
    dH = get_dH_factors(theta)
    
    I_xx = (p.M_D * (p.D_D/2)**2) / 4 
    
    # --- Mass/Inertia Matrix M (Eq. 21) ---
    M11 = (1/3)*p.M_B*(p.L**2)*H[0] + p.L*p.I_B*H[2] + p.M_D*(p.L**2)*H[4] + I_xx*H[6]
    M22 = (1/4)*p.M_B*(p.L**2)*H[1] + p.L*p.I_B*H[3] + p.M_D*(p.L**2)*H[5] + I_xx*H[7]
    M = np.array([
        [M11, 0.0],
        [0.0, M22]
    ])
    
    # --- Coriolis Matrix Coefficients C (Eq. 22) ---
    C11 = 0.5 * ((p.L**2)*p.M_B*dH[0] + p.L*p.I_B*dH[2] + (p.L**2)*p.M_D*dH[4] + I_xx*dH[6])
    C12, C21, C23 = 0.0, 0.0, 0.0
    C13 = -0.5 * ((p.L**2)*p.M_B*dH[1] + p.L*p.I_B*dH[3] + (p.L**2)*p.M_D*dH[5] + I_xx*dH[7])
    C22 = ((p.L**2)*p.M_B*dH[1] + p.L*p.I_B*dH[3] + (p.L**2)*p.M_D*dH[5] + I_xx*dH[7])
    
    # C is defined as multiplying against the velocity terms [dtheta^2, dphi^2, dtheta*dphi]^T
    C_matrix = np.array([
        [C11, C12, C13],
        [C21, C22, C23]
    ])
    
    # --- Stiffness Matrix K (Eq. 23) ---
    K = np.array([
        [(p.E * p.I_B) / p.L, 0.0],
        [0.0,                 0.0]
    ])
    
    # --- Actuation Map Matrix D (Eq. 24) ---
    # Corrected indexing D_31/D_32 to match matrix layout D_21/D_22 as given by Eq 19 & Eq 20
    D11 = p.R * np.cos(phi)
    D12 = p.R * np.cos((2*np.pi/3) - phi)
    D21 = -p.R * theta * np.sin(phi)
    D22 = p.R * theta * np.sin((2*np.pi/3) - phi)
    
    D = np.array([
        [D11, D12],
        [D21, D22]
    ])
    
    return M, C_matrix, K, D

def state_derivative(t, state, force_func):
    """
    The ODE function for scipy.integrate.solve_ivp.
    state = [theta, phi, dtheta, dphi]
    """
    theta, phi, dtheta, dphi = state
    
    # Avoid exact zero to prevent singular matrices/divisions during transient states
    if abs(theta) < 1e-6:
        theta = 1e-6
        
    F = force_func(t) # [F1, F2]
    
    M, C_mat, K, D = assemble_matrices(theta, phi)
    
    # Vector of generalized velocities: [dtheta^2, dphi^2, dtheta*dphi]^T
    v_vec = np.array([dtheta**2, dphi**2, dtheta*dphi])
    
    # Calculate forces
    Q_applied = D @ F
    Q_coriolis = C_mat @ v_vec
    Q_stiffness = K @ np.array([theta, phi])
    
    # M * q_ddot = Q_applied - Q_coriolis - Q_stiffness
    RHS = Q_applied - Q_coriolis - Q_stiffness
    
    # Solve for accelerations: q_ddot = M^-1 * RHS
    # Since M is diagonal, inversion is trivial and stable
    q_ddot = np.linalg.solve(M, RHS)
    
    return [dtheta, dphi, q_ddot[0], q_ddot[1]]



def assemble_matrices_with_payload(theta, phi, m_p):
    """
    Assembles M, C, K, D, and G matrices including the payload mass.
    """
    # Get base robot factors
    H = get_H_factors(theta)
    dH = get_dH_factors(theta)
    I_xx = (p.M_D * (p.D_D/2)**2) / 4 
    
    # Get payload factors
    M_p, C_p, G_p = get_payload_factors(theta, p.L, m_p)
    
    # --- Mass/Inertia Matrix M ---
    M11_base = (1/3)*p.M_B*(p.L**2)*H[0] + p.L*p.I_B*H[2] + p.M_D*(p.L**2)*H[4] + I_xx*H[6]
    M22_base = (1/4)*p.M_B*(p.L**2)*H[1] + p.L*p.I_B*H[3] + p.M_D*(p.L**2)*H[5] + I_xx*H[7]
    M_base = np.array([[M11_base, 0.0], [0.0, M22_base]])
    
    M_total = M_base + M_p  # Add payload inertia
    
    # --- Coriolis Matrix C ---
    C11_base = 0.5 * ((p.L**2)*p.M_B*dH[0] + p.L*p.I_B*dH[2] + (p.L**2)*p.M_D*dH[4] + I_xx*dH[6])
    C13_base = -0.5 * ((p.L**2)*p.M_B*dH[1] + p.L*p.I_B*dH[3] + (p.L**2)*p.M_D*dH[5] + I_xx*dH[7])
    C22_base = ((p.L**2)*p.M_B*dH[1] + p.L*p.I_B*dH[3] + (p.L**2)*p.M_D*dH[5] + I_xx*dH[7])
    
    C_base = np.array([
        [C11_base, 0.0, C13_base],
        [0.0, C22_base, 0.0]
    ])
    
    C_total = C_base + C_p  # Add payload Coriolis
    
    # --- Stiffness Matrix K ---
    K_total = np.array([
        [(p.E * p.I_B) / p.L, 0.0],
        [0.0,                 0.0]
    ])
    
    # --- Actuation Map Matrix D ---
    D_total = np.array([
        [p.R * np.cos(phi),          p.R * np.cos((2*np.pi/3) - phi)],
        [-p.R * theta * np.sin(phi), p.R * theta * np.sin((2*np.pi/3) - phi)]
    ])
    
    return M_total, C_total, K_total, D_total, G_p

def state_derivative_payload(t, state, force_func, m_p):
    """
    The updated ODE function supporting variable payload m_p.
    """
    theta, phi, dtheta, dphi = state
    if abs(theta) < 1e-6:
        theta = 1e-6
        
    F = force_func(t) 
    
    M, C_mat, K, D, G = assemble_matrices_with_payload(theta, phi, m_p)
    v_vec = np.array([dtheta**2, dphi**2, dtheta*dphi])
    
    # Calculate forces
    Q_applied = D @ F
    Q_coriolis = C_mat @ v_vec
    Q_stiffness = K @ np.array([theta, phi])
    
    # M * q_ddot + C*v + K*q + G = Q_applied  =>  M * q_ddot = Q_applied - C*v - K*q - G
    RHS = Q_applied - Q_coriolis - Q_stiffness - G
    
    q_ddot = np.linalg.solve(M, RHS)
    
    return [dtheta, dphi, q_ddot[0], q_ddot[1]]