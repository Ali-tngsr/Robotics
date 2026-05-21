import params as p
from taylor_factors import get_H_factors

def compute_energies(theta, phi, dtheta, dphi):
    """
    Computes T and U for the continuum robot.
    """
    H = get_H_factors(theta)
    
    # 1. Translational kinetic energy of backbone (Eq. 9)
    T_b_Trans = 0.5 * (p.L**2) * p.M_B * ((1/3) * H[0] * dtheta**2 + 0.25 * H[1] * dphi**2)
    
    # 2. Rotational kinetic energy of backbone (Eq. 14)
    T_b_Rot = 0.5 * p.L * p.I_B * (H[2] * dtheta**2 + H[3] * dphi**2)
    
    # 3. Translational kinetic energy of all disks (Eq. 15)
    T_d_Trans = 0.5 * (p.L**2) * p.M_D * (H[4] * dtheta**2 + H[5] * dphi**2)
    
    # 4. Rotational kinetic energy of all disks (Eq. 16)
    # Using I_xx as approximated for circular disks: I_xx = (m_d * r^2) / 4
    I_xx = (p.M_D * (p.D_D/2)**2) / 4 
    T_d_Rot = 0.5 * I_xx * (H[6] * dtheta**2 + H[7] * dphi**2)
    
    # Total Kinetic Energy (Eq. 6)
    T = T_b_Trans + T_b_Rot + T_d_Trans + T_d_Rot
    
    # Total Potential Energy - strictly elastic (Eq. 18)
    U = (p.E * p.I_B / (2 * p.L)) * theta**2
    
    return T, U