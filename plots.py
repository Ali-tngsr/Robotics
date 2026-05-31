"""
PLOTTING MODULE
Generate validation plots against paper figures.
"""
import numpy as np
import matplotlib.pyplot as plt
from params import L, THETA_MAX
from taylor_factors import (H1, H2, H3, H4, H5, H6, H7, H8,
                            _H1_exact, _H1_taylor, _H2_exact, _H2_taylor,
                            _H3_exact, _H3_taylor, _H4_exact, _H4_taylor)
from dynamics import (total_kinetic_energy, total_potential_energy,
                     mass_matrix, force_matrix)
from simulate import (simulate_static_equilibrium, simulate_fdr_example1,
                     simulate_fdr_example2, simulate_idr_example1,
                     simulate_idr_example2, simulate_pid_control)


def plot_H1_H2_comparison():
    """
    Figure 2: Exact vs Taylor expansion of H1, H2 and errors.
    """
    theta_vals = np.linspace(1e-4, THETA_MAX, 500)
    
    # Compute exact and Taylor
    h1_exact = np.array([_H1_exact(t) for t in theta_vals])
    h1_taylor = np.array([_H1_taylor(t) for t in theta_vals])
    h1_error = h1_exact - h1_taylor
    
    h2_exact = np.array([_H2_exact(t) for t in theta_vals])
    h2_taylor = np.array([_H2_taylor(t) for t in theta_vals])
    h2_error = h2_exact - h2_taylor
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    
    # H1 value
    axes[0, 0].plot(theta_vals, h1_exact, 'r-', linewidth=2.5, label='Exact')
    axes[0, 0].plot(theta_vals, h1_taylor, 'b-', linewidth=2.5, label='Taylor')
    axes[0, 0].set_ylabel('Values of H₁', fontsize=11)
    axes[0, 0].legend(fontsize=10)
    axes[0, 0].grid(True, alpha=0.3)
    
    # H1 error
    axes[1, 0].plot(theta_vals, h1_error, 'k-', linewidth=2)
    axes[1, 0].set_ylabel('Error', fontsize=11)
    axes[1, 0].set_xlabel('Bending angle θ (rad)', fontsize=11)
    axes[1, 0].grid(True, alpha=0.3)
    
    # H2 value
    axes[0, 1].plot(theta_vals, h2_exact, 'r-', linewidth=2.5, label='Exact')
    axes[0, 1].plot(theta_vals, h2_taylor, 'b-', linewidth=2.5, label='Taylor')
    axes[0, 1].set_ylabel('Values of H₂', fontsize=11)
    axes[0, 1].legend(fontsize=10)
    axes[0, 1].grid(True, alpha=0.3)
    
    # H2 error
    axes[1, 1].plot(theta_vals, h2_error, 'k-', linewidth=2)
    axes[1, 1].set_ylabel('Error', fontsize=11)
    axes[1, 1].set_xlabel('Bending angle θ (rad)', fontsize=11)
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.suptitle('Figure 2: H₁, H₂ — Exact vs Taylor Expansion', fontsize=13, fontweight='bold')
    plt.tight_layout()
    return fig


def plot_H3_H4_comparison():
    """Figure 3: H3, H4 comparison."""
    theta_vals = np.linspace(1e-4, THETA_MAX, 500)
    
    h3_exact = np.array([_H3_exact(t) for t in theta_vals])
    h3_taylor = np.array([_H3_taylor(t) for t in theta_vals])
    h3_error = h3_exact - h3_taylor
    
    h4_exact = np.array([_H4_exact(t) for t in theta_vals])
    h4_taylor = np.array([_H4_taylor(t) for t in theta_vals])
    h4_error = h4_exact - h4_taylor
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    
    axes[0, 0].plot(theta_vals, h3_exact, 'r-', linewidth=2.5, label='Exact')
    axes[0, 0].plot(theta_vals, h3_taylor, 'b-', linewidth=2.5, label='Taylor')
    axes[0, 0].set_ylabel('Values of H₃', fontsize=11)
    axes[0, 0].legend(fontsize=10)
    axes[0, 0].grid(True, alpha=0.3)
    
    axes[1, 0].plot(theta_vals, h3_error, 'k-', linewidth=2)
    axes[1, 0].set_ylabel('Error', fontsize=11)
    axes[1, 0].set_xlabel('Bending angle θ (rad)', fontsize=11)
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[0, 1].plot(theta_vals, h4_exact, 'r-', linewidth=2.5, label='Exact')
    axes[0, 1].plot(theta_vals, h4_taylor, 'b-', linewidth=2.5, label='Taylor')
    axes[0, 1].set_ylabel('Values of H₄', fontsize=11)
    axes[0, 1].legend(fontsize=10)
    axes[0, 1].grid(True, alpha=0.3)
    
    axes[1, 1].plot(theta_vals, h4_error, 'k-', linewidth=2)
    axes[1, 1].set_ylabel('Error', fontsize=11)
    axes[1, 1].set_xlabel('Bending angle θ (rad)', fontsize=11)
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.suptitle('Figure 3: H₃, H₄ — Exact vs Taylor Expansion', fontsize=13, fontweight='bold')
    plt.tight_layout()
    return fig


def plot_static_equilibrium():
    """Figure 8: Static equilibrium response."""
    print("  Running static equilibrium simulation...")
    t, y = simulate_static_equilibrium(t_final=40, num_points=1000)
    
    theta_deg = y[0] * 180 / np.pi
    phi_deg = y[1] * 180 / np.pi
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    
    ax1.plot(t, theta_deg, 'b-', linewidth=1.5)
    ax1.set_ylabel('θ (degrees)', fontsize=11)
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(-60, 60)
    ax1.axhline(0, color='k', linestyle='--', alpha=0.3)
    
    ax2.plot(t, phi_deg, 'r-', linewidth=1.5)
    ax2.set_ylabel('φ (degrees)', fontsize=11)
    ax2.set_xlabel('Time (sec)', fontsize=11)
    ax2.grid(True, alpha=0.3)
    
    plt.suptitle('Figure 8: Static Equilibrium (θ₀=π/4, φ₀=0, F=0)', 
                 fontsize=13, fontweight='bold')
    plt.tight_layout()
    return fig


def plot_fdr_example1():
    """Figure 9: Forward dynamics response (5N on cable 1)."""
    print("  Running FDR Example 1 simulation...")
    t, y = simulate_fdr_example1(t_final=40, F1=5.0)
    
    theta_deg = y[0] * 180 / np.pi
    phi_deg = y[1] * 180 / np.pi
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    
    ax1.plot(t, theta_deg, 'b-', linewidth=1.5)
    ax1.set_ylabel('θ (degrees)', fontsize=11)
    ax1.grid(True, alpha=0.3)
    ax1.axhline(15.53, color='g', linestyle='--', linewidth=2, alpha=0.7, label='Expected: 15.53°')
    ax1.legend(fontsize=9)
    
    ax2.plot(t, phi_deg, 'r-', linewidth=1.5)
    ax2.set_ylabel('φ (degrees)', fontsize=11)
    ax2.set_xlabel('Time (sec)', fontsize=11)
    ax2.grid(True, alpha=0.3)
    
    plt.suptitle('Figure 9: FDR Example 1 (F₁=5N, F₂=0N)', 
                 fontsize=13, fontweight='bold')
    plt.tight_layout()
    return fig


def plot_fdr_example2():
    """Figure 10: Forward dynamics with varying forces."""
    print("  Running FDR Example 2 simulation...")
    t, y = simulate_fdr_example2(t_final=10)
    
    theta = y[0]
    phi = y[1]
    
    # Calculate forces over time (matching simulate_fdr_example2)
    F1_input = 3.5 * t
    F2_input = 1.5 * t
    
    # Calculate Cartesian coordinates of the end-effector (X, Y, Z)
    x_cart = np.zeros_like(theta)
    y_cart = np.zeros_like(theta)
    z_cart = np.zeros_like(theta)
    
    for i in range(len(theta)):
        th = theta[i]
        ph = phi[i]
        # Prevent Singularity at theta -> 0
        if abs(th) < 1e-6:
            x_cart[i] = 0.0
            y_cart[i] = 0.0
            z_cart[i] = L
        else:
            x_cart[i] = (L / th) * (1 - np.cos(th)) * np.cos(ph)
            y_cart[i] = (L / th) * (1 - np.cos(th)) * np.sin(ph)
            z_cart[i] = (L / th) * np.sin(th)
    
    fig, axes = plt.subplots(2, 1, figsize=(11, 8))
    
    # Forces input (Fig. 10a)
    axes[0].plot(t, F1_input, 'g-', linewidth=2, label='F₁')
    axes[0].plot(t, F2_input, 'm-', linewidth=2, label='F₂')
    axes[0].set_ylabel('Cable Force (N)', fontsize=11)
    axes[0].set_xlabel('Time (sec)', fontsize=11)
    axes[0].legend(fontsize=10)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('(a) Temporal evolution of cable tensions', fontsize=11)
    
    # Cartesian coordinates (Fig. 10b)
    axes[1].plot(t, x_cart * 1000, 'b-', linewidth=2, label='X')
    axes[1].plot(t, y_cart * 1000, 'r-', linewidth=2, label='Y')
    axes[1].plot(t, z_cart * 1000, 'k-', linewidth=2, label='Z')
    axes[1].set_ylabel('Coordinates (mm)', fontsize=11)
    axes[1].set_xlabel('Time (sec)', fontsize=11)
    axes[1].legend(fontsize=10)
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('(b) Cartesian coordinates of the end-point', fontsize=11)
    
    plt.suptitle('Figure 10: FDR Example 2 (Varying Forces)', 
                 fontsize=13, fontweight='bold')
    plt.tight_layout()
    return fig


def plot_idr_example1():
    """Figure 12: Inverse dynamics tracking circular trajectory."""
    print("  Running IDR Example 1 simulation...")
    t, theta_d, phi_d, F_req = simulate_idr_example1()
    
    theta_deg = theta_d * 180 / np.pi
    phi_deg = phi_d * 180 / np.pi
    
    fig, axes = plt.subplots(2, 1, figsize=(11, 8))
    
    # Desired path
    # Create 2D projection: X-Z plane
    x_traj = (L/theta_d[0]) * (1 - np.cos(theta_d)) * np.cos(phi_d)
    z_traj = (L/theta_d[0]) * np.sin(theta_d)
    
    axes[0].plot(x_traj * 1000, z_traj * 1000, 'r-', linewidth=2, label='Desired path')
    axes[0].set_xlabel('X (mm)', fontsize=11)
    axes[0].set_ylabel('Z (mm)', fontsize=11)
    axes[0].set_title('(a) Desired path on 2-DOF CDCR workspace', fontsize=11)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=10)
    axes[0].axis('equal')
    
    # Required forces
    axes[1].plot(t, F_req[0], 'g-', linewidth=2, label='F₁')
    axes[1].plot(t, F_req[1], 'm-', linewidth=2, label='F₂')
    axes[1].set_ylabel('Force (N)', fontsize=11)
    axes[1].set_xlabel('Time (sec)', fontsize=11)
    axes[1].set_title('(b) Temporal evolution of actuation forces', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(fontsize=10)
    
    plt.suptitle('Figure 12: IDR Example 1 (Circular: θ=π/12, φ=(π/5)t)', 
                 fontsize=13, fontweight='bold')
    plt.tight_layout()
    return fig


def plot_idr_example2():
    """Figure 13: Inverse dynamics tracking angled trajectory."""
    print("  Running IDR Example 2 simulation...")
    t, theta_d, phi_d, F_req = simulate_idr_example2()
    
    theta_deg = theta_d * 180 / np.pi
    
    fig, axes = plt.subplots(2, 1, figsize=(11, 8))
    
    # Workspace projection
    # Handle theta ~0 at start carefully
    x_traj = np.zeros_like(theta_d)
    z_traj = np.zeros_like(theta_d)
    for i, theta in enumerate(theta_d):
        if abs(theta) > 1e-6:
            x_traj[i] = (L/theta) * (1 - np.cos(theta)) * np.cos(phi_d[i])
            z_traj[i] = (L/theta) * np.sin(theta)
    
    axes[0].plot(x_traj * 1000, z_traj * 1000, 'r-', linewidth=2, label='Desired path')
    axes[0].set_xlabel('X (mm)', fontsize=11)
    axes[0].set_ylabel('Z (mm)', fontsize=11)
    axes[0].set_title('(a) Desired path on 2-DOF CDCR workspace', fontsize=11)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=10)
    
    # Required forces
    axes[1].plot(t, F_req[0], 'g-', linewidth=2, label='F₁')
    axes[1].plot(t, F_req[1], 'm-', linewidth=2, label='F₂')
    axes[1].set_ylabel('Force (N)', fontsize=11)
    axes[1].set_xlabel('Time (sec)', fontsize=11)
    axes[1].set_title('(b) Temporal evolution of actuation forces', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(fontsize=10)
    
    plt.suptitle('Figure 13: IDR Example 2 (θ=(π/4)t, φ=π/6)', 
                 fontsize=13, fontweight='bold')
    plt.tight_layout()
    return fig


def plot_pid_control():
    """Figure 14: PID control response."""
    print("  Running PID control simulation...")
    t, hist, f_hist = simulate_pid_control(setpoint=15.53, t_final=5.0)
    
    theta_deg = hist[0] * 180 / np.pi
    phi_deg = hist[1] * 180 / np.pi
    F1 = f_hist[0]
    
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    
    # Theta
    axes[0].plot(t, theta_deg, 'b-', linewidth=2)
    axes[0].axhline(15.53, color='g', linestyle='-', linewidth=1.5, alpha=0.7, label='Target: 15.53°')
    axes[0].set_ylabel('θ (degrees)', fontsize=11)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=10)
    axes[0].set_ylim(0, 30)
    
    # Phi
    axes[1].plot(t, phi_deg, 'r-', linewidth=2)
    axes[1].set_ylabel('φ (degrees)', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    
    # Force
    axes[2].plot(t, F1, 'k-', linewidth=2)
    axes[2].set_ylabel('F₁ (N)', fontsize=11)
    axes[2].set_xlabel('Time (sec)', fontsize=11)
    axes[2].grid(True, alpha=0.3)
    axes[2].set_ylim(0, 10)
    
    plt.suptitle('Figure 14: Dynamic Response with PID Controller (Kp=2.8, Ki=0.004, Kd=0.38)',
                 fontsize=13, fontweight='bold')
    plt.tight_layout()
    return fig


# ─────────────────────────────────────────────────────────────────────────────
# MAIN EXECUTION
# ─────────────────────────────────────────────────────────────────────────────

def generate_all_plots(show=True, save=False):
    """Generate and optionally display/save all figures."""
    print("\n" + "="*70)
    print("PHASE 5+ VALIDATION: Generating Figures from Amouri et al. 2020")
    print("="*70 + "\n")
    
    figs = {}
    
    print("Figure 2 (H₁, H₂)...")
    figs['fig2'] = plot_H1_H2_comparison()
    if save:
        figs['fig2'].savefig('fig2_H1_H2.png', dpi=150, bbox_inches='tight')
    
    print("Figure 3 (H₃, H₄)...")
    figs['fig3'] = plot_H3_H4_comparison()
    if save:
        figs['fig3'].savefig('fig3_H3_H4.png', dpi=150, bbox_inches='tight')
    
    print("Figure 8 (Static Equilibrium)...")
    figs['fig8'] = plot_static_equilibrium()
    if save:
        figs['fig8'].savefig('fig8_static_eq.png', dpi=150, bbox_inches='tight')
    
    print("Figure 9 (FDR Example 1)...")
    figs['fig9'] = plot_fdr_example1()
    if save:
        figs['fig9'].savefig('fig9_fdr1.png', dpi=150, bbox_inches='tight')
    
    print("Figure 10 (FDR Example 2)...")
    figs['fig10'] = plot_fdr_example2()
    if save:
        figs['fig10'].savefig('fig10_fdr2.png', dpi=150, bbox_inches='tight')
    
    print("Figure 12 (IDR Example 1)...")
    figs['fig12'] = plot_idr_example1()
    if save:
        figs['fig12'].savefig('fig12_idr1.png', dpi=150, bbox_inches='tight')
    
    print("Figure 13 (IDR Example 2)...")
    figs['fig13'] = plot_idr_example2()
    if save:
        figs['fig13'].savefig('fig13_idr2.png', dpi=150, bbox_inches='tight')
    
    print("Figure 14 (PID Control)...")
    figs['fig14'] = plot_pid_control()
    if save:
        figs['fig14'].savefig('fig14_pid.png', dpi=150, bbox_inches='tight')
    
    if show:
        plt.show()
    
    print("\n" + "="*70)
    print("✓ All plots generated successfully!")
    print("="*70 + "\n")
    
    return figs


if __name__ == '__main__':
    figs = generate_all_plots(show=True, save=False)
