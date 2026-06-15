"""
PLOTTING MODULE
Generate validation plots against paper figures.
Supports dynamic payload injection (m_p) for comparison.
(Refactored: Centralized imports, fixed duplicates, added m_p support)
"""
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

# ─────────────────────────────────────────────────────────────────────────────
# IMPORTS FROM PROJECT MODULES
# ─────────────────────────────────────────────────────────────────────────────
from params import L, THETA_MAX
from taylor_factors import (H1, H2, H3, H4, H5, H6, H7, H8,
                            _H1_exact, _H1_taylor, _H2_exact, _H2_taylor,
                            _H3_exact, _H3_taylor, _H4_exact, _H4_taylor)
from dynamics import total_kinetic_energy, total_potential_energy, mass_matrix, force_matrix
from simulate import (simulate_static_equilibrium, simulate_fdr_example1,
                     simulate_fdr_example2, simulate_idr_example1,
                     simulate_idr_example2, simulate_pid_control)


# ─────────────────────────────────────────────────────────────────────────────
# PUBLICATION-QUALITY PLOT SETTINGS (LaTeX Style)
# ─────────────────────────────────────────────────────────────────────────────
mpl.rcParams['mathtext.fontset'] = 'cm'
mpl.rcParams['font.family'] = 'serif'
mpl.rcParams['font.serif'] = ['Computer Modern Roman', 'Times New Roman', 'DejaVu Serif']

# mpl.rcParams['text.usetex'] = True  # در صورت داشتن LaTeX روی سیستم فعال کنید

mpl.rcParams['axes.titlesize'] = 12
mpl.rcParams['axes.labelsize'] = 11
mpl.rcParams['legend.fontsize'] = 10
mpl.rcParams['xtick.labelsize'] = 10
mpl.rcParams['ytick.labelsize'] = 10


# ─────────────────────────────────────────────────────────────────────────────
# PLOT FUNCTIONS
# ─────────────────────────────────────────────────────────────────────────────

def plot_H1_H2_comparison():
    """Figure 2: Exact vs Taylor expansion of H1, H2 and errors."""
    theta_vals = np.linspace(1e-4, THETA_MAX, 500)
    
    h1_exact = np.array([_H1_exact(t) for t in theta_vals])
    h1_taylor = np.array([_H1_taylor(t) for t in theta_vals])
    h1_error = h1_exact - h1_taylor
    
    h2_exact = np.array([_H2_exact(t) for t in theta_vals])
    h2_taylor = np.array([_H2_taylor(t) for t in theta_vals])
    h2_error = h2_exact - h2_taylor
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    
    axes[0, 0].plot(theta_vals, h1_exact, 'r-', linewidth=2.5, label='Exact')
    axes[0, 0].plot(theta_vals, h1_taylor, 'b-', linewidth=2.5, label='Taylor')
    axes[0, 0].set_ylabel('Values of H₁', fontsize=11)
    axes[0, 0].legend(fontsize=10)
    axes[0, 0].grid(True, alpha=0.3)
    
    axes[1, 0].plot(theta_vals, h1_error, 'k-', linewidth=2)
    axes[1, 0].set_ylabel('Error', fontsize=11)
    axes[1, 0].set_xlabel('Bending angle θ (rad)', fontsize=11)
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[0, 1].plot(theta_vals, h2_exact, 'r-', linewidth=2.5, label='Exact')
    axes[0, 1].plot(theta_vals, h2_taylor, 'b-', linewidth=2.5, label='Taylor')
    axes[0, 1].set_ylabel('Values of H₂', fontsize=11)
    axes[0, 1].legend(fontsize=10)
    axes[0, 1].grid(True, alpha=0.3)
    
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


def plot_static_equilibrium(m_p=0.0):
    """Figure 8: Static equilibrium response."""
    print(f"  Running static equilibrium simulation (m_p={m_p})...")
    t, y = simulate_static_equilibrium(t_final=40, num_points=1000, m_p=m_p)
    
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
    
    plt.suptitle(f'Figure 8: Static Equilibrium (θ₀=π/4, F=0) | Payload: {m_p} kg', 
                 fontsize=13, fontweight='bold')
    plt.tight_layout()
    return fig


def plot_fdr_example1(m_p=0.0):
    """Figure 9: Forward dynamics response (5N on cable 1)."""
    print(f"  Running FDR Example 1 simulation (m_p={m_p})...")
    t, y = simulate_fdr_example1(t_final=40, F1=5.0, m_p=m_p)
    
    theta_deg = y[0] * 180 / np.pi
    phi_deg = y[1] * 180 / np.pi
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    
    ax1.plot(t, theta_deg, 'b-', linewidth=1.5)
    ax1.set_ylabel('θ (degrees)', fontsize=11)
    ax1.grid(True, alpha=0.3)
    
    # خط مرجع مقاله (بدون بار)
    if m_p == 0.0:
        ax1.axhline(15.53, color='g', linestyle='--', linewidth=2, alpha=0.7, label='Expected Baseline: 15.53°')
        ax1.legend(fontsize=9)
    
    ax2.plot(t, phi_deg, 'r-', linewidth=1.5)
    ax2.set_ylabel('φ (degrees)', fontsize=11)
    ax2.set_xlabel('Time (sec)', fontsize=11)
    ax2.grid(True, alpha=0.3)
    
    plt.suptitle(f'Figure 9: FDR Example 1 (F₁=5N) | Payload: {m_p} kg', 
                 fontsize=13, fontweight='bold')
    plt.tight_layout()
    return fig


def plot_fdr_example2(m_p=0.0):
    """Figure 10: Forward dynamics with varying forces & Cartesian coords."""
    print(f"  Running FDR Example 2 simulation (m_p={m_p})...")
    t, state, F1_vals, F2_vals, F3_vals = simulate_fdr_example2(t_final=10.0, m_p=m_p)
    
    theta = state[0, :]
    phi = state[1, :]
    
    X_mm = np.zeros_like(theta)
    Z_mm = np.zeros_like(theta)
    
    for i in range(len(theta)):
        th = theta[i]
        ph = phi[i]
        if abs(th) < 1e-6:
            X_mm[i] = 0.0
            Z_mm[i] = L * 1000.0
        else:
            X_mm[i] = (L / th) * (1 - np.cos(th)) * np.cos(ph) * 1000.0
            Z_mm[i] = (L / th) * np.sin(th) * 1000.0

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    
    ax1.plot(t, F1_vals, 'k-', linewidth=1.5, label='F1')
    ax1.plot(t, F2_vals, 'b--', linewidth=1.5, label='F2')
    ax1.plot(t, F3_vals, 'r-.', linewidth=1.5, label='F3')
    ax1.set_title("Temporal evolution of cable tensions")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Tension (N)")
    ax1.grid(True)
    ax1.legend()
    
    ax2.plot(X_mm, Z_mm, 'k-', linewidth=2)
    ax2.set_title("Cartesian coordinates of the end-point")
    ax2.set_xlabel("X (mm)")
    ax2.set_ylabel("Z (mm)")
    
    ax2.set_xlim([0, 600])
    ax2.set_ylim([380, 830]) 
    ax2.grid(True)
    
    plt.suptitle(f'Figure 10: FDR Example 2 (Varying Forces) | Payload: {m_p} kg', fontweight='bold')
    plt.tight_layout()
    return fig


def plot_idr_example1(m_p=0.0):
    """Figure 12: Inverse Dynamics - Circular Path."""
    print(f"  Running IDR Example 1 simulation (m_p={m_p})...")
    t, theta_d, phi_d, F_req = simulate_idr_example1(m_p=m_p)
    
    fig = plt.figure(figsize=(13, 6))
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    
    X = np.zeros_like(theta_d)
    Y = np.zeros_like(theta_d)
    Z = np.zeros_like(theta_d)
    
    for i in range(len(theta_d)):
        th = theta_d[i]
        ph = phi_d[i]
        if abs(th) < 1e-6:
            X[i], Y[i], Z[i] = 0.0, 0.0, L * 1000.0
        else:
            X[i] = (L / th) * (1 - np.cos(th)) * np.cos(ph) * 1000.0
            Y[i] = (L / th) * (1 - np.cos(th)) * np.sin(ph) * 1000.0
            Z[i] = (L / th) * np.sin(th) * 1000.0
            
    ax1.plot(X, Y, Z, 'b-', linewidth=2.5)
    ax1.set_title("(a) Desired path on CDCR workspace")
    ax1.set_xlabel("X (mm)")
    ax1.set_ylabel("Y (mm)")
    ax1.set_zlabel("Z (mm)")
    
    ax1.set_xlim([-150, 150])
    ax1.set_ylim([-150, 150])
    ax1.set_zlim([750, 800]) 
    
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(t, F_req[0], 'k-', linewidth=1.5, label='F1')
    ax2.plot(t, F_req[1], 'b--', linewidth=1.5, label='F2')
    ax2.plot(t, F_req[2], 'r-.', linewidth=1.5, label='F3')
    
    ax2.set_title("(b) Temporal evolution of the actuation forces")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Tension (N)")
    ax2.grid(True)
    ax2.legend()
    
    plt.suptitle(f'Figure 12: Inverse Dynamics (Circular) | Payload: {m_p} kg', fontweight='bold')
    plt.tight_layout()
    return fig


def plot_idr_example2(m_p=0.0):
    """Figure 13: Inverse Dynamics - Linear Path."""
    print(f"  Running IDR Example 2 simulation (m_p={m_p})...")
    t, theta_d, phi_d, F_req = simulate_idr_example2(m_p=m_p)
    
    fig = plt.figure(figsize=(13, 6))
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    
    X = np.zeros_like(theta_d)
    Y = np.zeros_like(theta_d)
    Z = np.zeros_like(theta_d)
    
    for i in range(len(theta_d)):
        th = theta_d[i]
        ph = phi_d[i]
        if abs(th) < 1e-6:
            X[i], Y[i], Z[i] = 0.0, 0.0, L * 1000.0
        else:
            X[i] = (L / th) * (1 - np.cos(th)) * np.cos(ph) * 1000.0
            Y[i] = (L / th) * (1 - np.cos(th)) * np.sin(ph) * 1000.0
            Z[i] = (L / th) * np.sin(th) * 1000.0
            
    ax1.plot(X, Y, Z, 'b-', linewidth=2.5)
    ax1.set_title("(a) Desired linear path on CDCR workspace")
    ax1.set_xlabel("X (mm)")
    ax1.set_ylabel("Y (mm)")
    ax1.set_zlabel("Z (mm)")
    
    ax1.set_xlim([0, 300])
    ax1.set_ylim([0, 300])
    ax1.set_zlim([700, 850])
    
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(t, F_req[0], 'k-', linewidth=1.5, label='F1')
    ax2.plot(t, F_req[1], 'b--', linewidth=1.5, label='F2')
    ax2.plot(t, F_req[2], 'r-.', linewidth=1.5, label='F3')
    
    ax2.set_title("(b) Temporal evolution of actuation forces")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Tension (N)")
    ax2.grid(True)
    ax2.legend()
    
    plt.suptitle(f'Figure 13: Inverse Dynamics (Linear) | Payload: {m_p} kg', fontweight='bold')
    plt.tight_layout()
    return fig


def plot_pid_control(m_p=0.0):
    """Figure 14: PID Control Response."""
    print(f"  Running PID Control simulation (m_p={m_p})...")
    t, state_history, force_history = simulate_pid_control(setpoint=15.53, t_final=5.0, dt=0.01, m_p=m_p)
    
    theta_deg = state_history[0, :] * 180.0 / np.pi
    phi_deg = state_history[1, :] * 180.0 / np.pi
    F1 = force_history[0, :]
    
    fig, axes = plt.subplots(3, 1, figsize=(9, 10))
    
    # --- Subplot 1: Theta ---
    axes[0].plot(t, theta_deg, 'k-', linewidth=2, label=r'Bending angle $\theta(t)$')
    axes[0].axhline(y=15.53, color='r', linestyle='--', linewidth=1.5, label=r'Target $\theta_{ref} = 15.53^\circ$')
    axes[0].set_title(r"Dynamic response for the bending angle $\theta$", fontweight='bold')
    axes[0].set_ylabel(r"Angle $(^\circ)$")
    axes[0].set_ylim([0, 18]) 
    axes[0].set_xlim([0, 5])
    axes[0].grid(True, linestyle=':', alpha=0.7)
    axes[0].legend(loc='lower right')
    
    # --- Subplot 2: Phi ---
    axes[1].plot(t, phi_deg, 'b-', linewidth=2, label=r'Orientation angle $\phi(t)$')
    axes[1].axhline(y=0.0, color='r', linestyle='--', linewidth=1.5, label=r'Target $\phi_{ref} = 0^\circ$')
    axes[1].set_title(r"Dynamic response for the orientation angle $\phi$", fontweight='bold')
    axes[1].set_ylabel(r"Angle $(^\circ)$")
    axes[1].set_ylim([-0.5, 0.5]) 
    axes[1].set_xlim([0, 5])
    axes[1].grid(True, linestyle=':', alpha=0.7)
    axes[1].legend(loc='upper right')
    
    # --- Subplot 3: Force F1 ---
    axes[2].plot(t, F1, 'g-', linewidth=2, label=r'Actuation Force $F_1(t)$')
    axes[2].axhline(y=5.0, color='r', linestyle='--', linewidth=1.5, label=r'Steady-state $F_{ss} \approx 5$ N')
    axes[2].set_title(r"Temporal evolution of the actuation force $F_1$", fontweight='bold')
    axes[2].set_xlabel(r"Time (s)")
    axes[2].set_ylabel(r"Force (N)")
    axes[2].set_ylim([0, 8]) 
    axes[2].set_xlim([0, 5])
    axes[2].grid(True, linestyle=':', alpha=0.7)
    axes[2].legend(loc='upper right')
    
    plt.suptitle(f'Figure 14: PID Control Response | Payload: {m_p} kg', fontsize=13, fontweight='bold')
    plt.tight_layout()
    return fig


# ─────────────────────────────────────────────────────────────────────────────
# MAIN EXECUTION
# ─────────────────────────────────────────────────────────────────────────────

def generate_all_plots(show=True, save=False, m_p=0.0):
    """Generate and optionally display/save all figures for a given payload (m_p)."""
    print("\n" + "="*70)
    print(f"PHASE 5+ VALIDATION: Generating Figures (m_p = {m_p} kg)")
    print("="*70 + "\n")
    
    figs = {}
    
    print("Figure 2 (H₁, H₂)...")
    figs['fig02_H1_H2'] = plot_H1_H2_comparison()
    
    print("Figure 3 (H₃, H₄)...")
    figs['fig03_H3_H4'] = plot_H3_H4_comparison()
    
    print("Figure 8 (Static Equilibrium)...")
    figs['fig08_static_equilibrium'] = plot_static_equilibrium(m_p=m_p)
    
    print("Figure 9 (FDR Example 1)...")
    figs['fig09_fdr_example1'] = plot_fdr_example1(m_p=m_p)
    
    print("Figure 10 (FDR Example 2)...")
    figs['fig10_fdr_example2'] = plot_fdr_example2(m_p=m_p)
    
    print("Figure 12 (IDR Example 1)...")
    figs['fig12_idr_example1'] = plot_idr_example1(m_p=m_p)
    
    print("Figure 13 (IDR Example 2)...")
    figs['fig13_idr_example2'] = plot_idr_example2(m_p=m_p)
    
    print("Figure 14 (PID Control)...")
    figs['fig14_pid_control'] = plot_pid_control(m_p=m_p)
    
    if save:
        print("\nSaving high-quality plots...")
        # ساخت یک ساب‌فولدر بر اساس وزن برای جلوگیری از تداخل عکس‌ها
        save_dir = os.path.join("figures", f"payload_{int(m_p*1000)}g")
        os.makedirs(save_dir, exist_ok=True)
        
        for fig_name, fig in figs.items():
            if fig is not None:
                filepath = os.path.join(save_dir, f"{fig_name}.png")
                fig.savefig(filepath, dpi=300, bbox_inches='tight', format='png')
                print(f"  -> Saved: {filepath}")

    if show:
        plt.show()
    
    print("\n" + "="*70)
    print("✓ All plots generated successfully!")
    print("="*70 + "\n")
    
    return figs


if __name__ == '__main__':
    # در حالت دیفالت ربات بدون بار را رسم می‌کند
    figs = generate_all_plots(show=True, save=True, m_p=0.0)
