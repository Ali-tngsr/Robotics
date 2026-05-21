import numpy as np

# Table 1: Estimated parameters and geometric properties of 2-DOF CDCR 
L = 0.802          # Length of flexible backbone (m) [cite: 360, 384]
M_B = 0.0326       # Mass of flexible backbone (kg) [cite: 361, 385]
M_D = 0.0082       # Disk mass (kg) [cite: 362, 386]
D_B = 0.005        # Diameter of flexible backbone (m) [cite: 363, 387]
D_D = 0.04         # Diameter of disk (m) [cite: 364, 388]
R = 0.019          # Radial distance between cables and neutral axis (m) [cite: 365, 389]
E = 9.5e9          # Elasticity modulus (Pa) [cite: 366, 390]

# Derived parameters
I_B = (np.pi * D_B**4) / 64  # Second moment of area for a circular backbone