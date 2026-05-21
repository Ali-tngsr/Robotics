import numpy as np

def get_payload_factors(theta, L, m_p, g=9.81):
    """
    Returns the Taylor-approximated Inertia, Coriolis, and Gravity 
    contributions of a point-mass payload at the end-effector.
    """
    # 1. Payload Inertia Matrix elements (M_p)
    # Derived from (dx/dtheta)^2 + (dy/dtheta)^2 + (dz/dtheta)^2
    M_p11 = m_p * (L**2) * (1/4 - (theta**2)/72 + (theta**4)/3200)
    
    # Derived from (dx/dphi)^2 + (dy/dphi)^2
    M_p22 = m_p * (L**2) * ((theta**2)/4 - (theta**4)/48)
    
    # 2. Payload Coriolis Matrix elements (C_p)
    # Derived from derivatives of M_p with respect to theta
    dM_p11_dtheta = m_p * (L**2) * (-theta/36 + (theta**3)/800)
    dM_p22_dtheta = m_p * (L**2) * (theta/2 - (theta**3)/12)
    
    C_p11 = 0.5 * dM_p11_dtheta
    C_p12 = 0.0
    C_p13 = -0.5 * dM_p22_dtheta
    C_p21 = 0.0
    C_p22 = dM_p22_dtheta
    C_p23 = 0.0
    
    # 3. Payload Gravity Vector (G_p)
    # Derived from U_p = m_p * g * z_ee
    # G_p1 = d(U_p)/dtheta. Note: d(U_p)/dphi is 0.
    G_p1 = m_p * g * L * (-theta/3 + (theta**3)/30)
    G_p2 = 0.0
    
    # Pack into matrices
    M_p = np.array([
        [M_p11, 0.0],
        [0.0, M_p22]
    ])
    
    C_p = np.array([
        [C_p11, C_p12, C_p13],
        [C_p21, C_p22, C_p23]
    ])
    
    G_p = np.array([G_p1, G_p2])
    
    return M_p, C_p, G_p