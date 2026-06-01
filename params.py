"""
Physical parameters of 2-DOF CDCR (Table 1, Amouri et al. 2020).
All in SI units (m, kg, Pa, etc).
"""
import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# GEOMETRY
# ─────────────────────────────────────────────────────────────────────────────
L      = 0.802              # m  — total length of flexible backbone
r_cab  = 0.019              # m  — radial distance (cable to neutral axis)
d_b    = 0.005              # m  — backbone diameter (circular)
d_d    = 0.040              # m  — disk diameter (circular)

# ─────────────────────────────────────────────────────────────────────────────
# MASS / INERTIA
# ─────────────────────────────────────────────────────────────────────────────
m_b    = 0.0326             # kg — backbone mass
m_d    = 0.0082             # kg — per-disk mass (×10 disks total)
N_d    = 10                 # — number of disks

# ─────────────────────────────────────────────────────────────────────────────
# MATERIAL
# ─────────────────────────────────────────────────────────────────────────────
E      = 9.5e9              # Pa — Young's modulus (elasticity)

# ─────────────────────────────────────────────────────────────────────────────
# DERIVED CROSS-SECTIONAL PROPERTIES (circular sections)
# ─────────────────────────────────────────────────────────────────────────────
I_b = np.pi * d_b**4 / 64   # m^4 — second moment of area (backbone)
I_d = np.pi * d_d**4 / 64   # m^4 — second moment of area (per disk)

# Disk moment of inertia (solid cylinder, about diameter axis)
# I_xx = I_yy = (1/4)*m*R^2,  I_zz = (1/2)*m*R^2
R_d = d_d / 2
I_xx = 0.25 * m_d * R_d**2  # kg·m^2 — about X or Y axis (local frame)
I_yy = I_xx
I_zz = 0.50 * m_d * R_d**2  # kg·m^2 — about Z axis (local frame)

# ─────────────────────────────────────────────────────────────────────────────
# OPERATING RANGE (weak bending angle assumption)
# ─────────────────────────────────────────────────────────────────────────────
THETA_MAX = 3 * np.pi / 5   # rad ≈ 1.884 (≈108°)
THETA_MIN = -THETA_MAX

# ─────────────────────────────────────────────────────────────────────────────
# PAYLOAD & ENVIRONMENT (Added for Extension Phase)
# ─────────────────────────────────────────────────────────────────────────────
m_p = 0.150                 # kg — payload mass (150 grams for example)
g   = 9.81                  # m/s^2 — gravitational acceleration
