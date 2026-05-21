import numpy as np
import sympy as sp
import matplotlib.pyplot as plt

# 1. Define symbolic variables
theta_sym = sp.Symbol('theta')


# Exact Analytical Expressions (Eq 10, 11, 25-30)
H1_exact = (theta_sym**3 + 6*theta_sym - 12*sp.sin(theta_sym) + 6*theta_sym*sp.cos(theta_sym)) / (theta_sym**5)
H2_exact = (6*theta_sym - 8*sp.sin(theta_sym) + sp.sin(2*theta_sym)) / (theta_sym**3)
H3_exact = 5/8 - 15/(64*theta_sym**2) + (sp.cos(theta_sym)**2)/(2*theta_sym**2) - sp.sin(2*theta_sym)/(8*theta_sym**3) - sp.sin(4*theta_sym)/(256*theta_sym**3)
H4_exact = 1/2 - (theta_sym * sp.sin(2*theta_sym))/(4*theta_sym**2)

H5_exact = (1/(40*theta_sym**4)) * (400 - 40*sp.cos(theta_sym/2) - 40*sp.cos(theta_sym/5) - 40*sp.cos(2*theta_sym/5) - 40*sp.cos(3*theta_sym/5) - 40*sp.cos(4*theta_sym/5) - 40*sp.cos(theta_sym/10) - 40*sp.cos(3*theta_sym/10) - 40*sp.cos(7*theta_sym/10) - 40*sp.cos(9*theta_sym/10) + 77*theta_sym**2 - 40*sp.cos(theta_sym) - 40*theta_sym*sp.sin(theta_sym) - 20*theta_sym*sp.sin(theta_sym/2) - 8*theta_sym*sp.sin(theta_sym/5) - 16*theta_sym*sp.sin(2*theta_sym/5) - 24*theta_sym*sp.sin(3*theta_sym/5) - 32*theta_sym*sp.sin(4*theta_sym/5) - 4*theta_sym*sp.sin(theta_sym/10) - 12*theta_sym*sp.sin(3*theta_sym/10) - 28*theta_sym*sp.sin(7*theta_sym/10) - 36*theta_sym*sp.sin(9*theta_sym/10))

# Note: Typo correction from paper text +c(9\theta/5) formatted as +(9\theta/5)
H6_exact = (1/(4*theta_sym**2)) * (30 + sp.cos(2*theta_sym) - 4*sp.cos(theta_sym/2) - 3*sp.cos(theta_sym/5) - 3*sp.cos(2*theta_sym/5) - 3*sp.cos(3*theta_sym/5) - 3*sp.cos(4*theta_sym/5) - 4*sp.cos(theta_sym/10) + sp.cos(6*theta_sym/5) - 4*sp.cos(3*theta_sym/10) + sp.cos(8*theta_sym/5) + sp.cos(9*theta_sym/5) - 4*sp.cos(7*theta_sym/10) - 4*sp.cos(9*theta_sym/10) - 3*sp.cos(theta_sym))

H7_exact = (1/32)*sp.cos(2*theta_sym) - (99/200)*sp.cos(theta_sym/5) - (303/800)*sp.cos(2*theta_sym/5) - (91/200)*sp.cos(3*theta_sym/5) - (17/50)*sp.cos(4*theta_sym/5) - (207/800)*sp.cos(6*theta_sym/5) - (51/200)*sp.cos(7*theta_sym/5) - (27/200)*sp.cos(8*theta_sym/5) - (19/200)*sp.cos(9*theta_sym/5) + (1/50)*sp.cos(12*theta_sym/5) + (9/800)*sp.cos(14*theta_sym/5) + (1/200)*sp.cos(16*theta_sym/5) + (1/800)*sp.cos(18*theta_sym/5) - (3/8)*sp.cos(theta_sym) + 1051/160

H8_exact = sp.sin(theta_sym/2)**2 + sp.sin(theta_sym/5)**2 + sp.sin(2*theta_sym/5)**2 + sp.sin(3*theta_sym/5)**2 + sp.sin(4*theta_sym/5)**2 + sp.sin(theta_sym/10)**2 + sp.sin(3*theta_sym/10)**2 + sp.sin(7*theta_sym/10)**2 + sp.sin(9*theta_sym/10)**2 + sp.sin(theta_sym)**2

H_exact_list = [H1_exact, H2_exact, H3_exact, H4_exact, H5_exact, H6_exact, H7_exact, H8_exact]

# Generate Taylor Expansions up to O(theta^4) and their derivatives
H_taylor_funcs = []
dH_taylor_funcs = []

for H in H_exact_list:
    # 6th order expansion gives up to theta^4 terms safely, removeO() drops the Big-O notation
    taylor_poly = sp.series(H, theta_sym, 0, 6).removeO() 
    H_taylor_funcs.append(sp.lambdify(theta_sym, taylor_poly, 'numpy'))
    
    # Derivative with respect to theta for the Coriolis matrix C
    dtaylor_poly = sp.diff(taylor_poly, theta_sym)
    dH_taylor_funcs.append(sp.lambdify(theta_sym, dtaylor_poly, 'numpy'))

def get_H_factors(theta):
    """Returns H1 to H8 evaluated at a specific theta"""
    return np.array([f(theta) for f in H_taylor_funcs])

def get_dH_factors(theta):
    """Returns dH1/dtheta to dH8/dtheta evaluated at a specific theta"""
    return np.array([f(theta) for f in dH_taylor_funcs]) 
H1_exact_sym = (theta_sym**3 + 6*theta_sym - 12*sp.sin(theta_sym) + 6*theta_sym*sp.cos(theta_sym)) / (theta_sym**5)
H2_exact_sym = (6*theta_sym - 8*sp.sin(theta_sym) + sp.sin(2*theta_sym)) / (theta_sym**3)

# 3. Generate Taylor Expansions dynamically using SymPy
# Expanding around theta = 0, up to O(theta^4)
# The paper states Taylor expansions approximate factors with small errors for theta in [-3pi/5, 3pi/5] [cite: 158]
H1_taylor_sym = sp.series(H1_exact_sym, theta_sym, 0, 6).removeO()
H2_taylor_sym = sp.series(H2_exact_sym, theta_sym, 0, 6).removeO()

# Convert symbolic functions to numerical functions for fast plotting
H1_exact_func = sp.lambdify(theta_sym, H1_exact_sym, 'numpy')
H2_exact_func = sp.lambdify(theta_sym, H2_exact_sym, 'numpy')
H1_taylor_func = sp.lambdify(theta_sym, H1_taylor_sym, 'numpy')
H2_taylor_func = sp.lambdify(theta_sym, H2_taylor_sym, 'numpy')

def plot_factor_comparison():
    """
    Replicates Figure 2: Comparison of exact and equivalent factors of H1 and H2[cite: 201].
    """
    # Range specified in the paper: up to ~1.88 rad (approx 3pi/5) [cite: 158]
    # We start slightly above 0 to avoid DivisionByZero in exact expressions
    theta_vals = np.linspace(1e-4, 3*np.pi/5, 500)
    
    h1_exact = H1_exact_func(theta_vals)
    h1_taylor = H1_taylor_func(theta_vals)
    h1_error = h1_exact - h1_taylor
    
    h2_exact = H2_exact_func(theta_vals)
    h2_taylor = H2_taylor_func(theta_vals)
    h2_error = h2_exact - h2_taylor

    fig, axs = plt.subplots(2, 2, figsize=(12, 8), gridspec_kw={'height_ratios': [3, 1]})
    
    # Plot H1
    axs[0, 0].plot(theta_vals, h1_exact, 'r-', linewidth=2, label='With analytical expressions')
    axs[0, 0].plot(theta_vals, h1_taylor, 'b-', linewidth=2, label='With Taylor expansions')
    axs[0, 0].set_ylabel('Values of H1')
    axs[0, 0].legend()
    axs[0, 0].grid(True)
    
    axs[1, 0].plot(theta_vals, h1_error, 'k-', linewidth=2)
    axs[1, 0].set_ylabel('Error')
    axs[1, 0].set_xlabel('The bending angle theta (rad)')
    axs[1, 0].grid(True)

    # Plot H2
    axs[0, 1].plot(theta_vals, h2_exact, 'r-', linewidth=2, label='With analytical expressions')
    axs[0, 1].plot(theta_vals, h2_taylor, 'b-', linewidth=2, label='With Taylor expansions')
    axs[0, 1].set_ylabel('Values of H2')
    axs[0, 1].legend()
    axs[0, 1].grid(True)
    
    axs[1, 1].plot(theta_vals, h2_error, 'k-', linewidth=2)
    axs[1, 1].set_ylabel('Error')
    axs[1, 1].set_xlabel('The bending angle theta (rad)')
    axs[1, 1].grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_factor_comparison()