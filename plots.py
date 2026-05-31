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
    
    # Extract end-effector position from final state
    theta_final = y[0, -1]
    phi_final = y[1, -1]
    
    theta_deg = y[0] * 180 / np.pi
    phi_deg = y[1] * 180 / np.pi
    
    fig, axes = plt.subplots(2, 1, figsize=(11, 8))
    
    # Forces input
    F1_input = 3.5 * t
    axes[0].plot(t, F1_input, 'r-', linewidth=2, label='F₁')
    axes[0].axhline(0, color='b', linestyle='--', linewidth=1.5, label='F₂=0')
    axes[0].set_ylabel('Cable Force (N)', fontsize=11)
    axes[0].set_xlabel('Time (sec)', fontsize=11)
    axes[0].legend(fontsize=10)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('(a) Temporal evolution of cable tensions', fontsize=11)
    
    # Angles
    axes[1].plot(t, theta_deg, 'b-', linewidth=2, label='θ')
    ax1_twin = axes[1].twinx()
    ax1_twin.plot(t, phi_deg, 'r-', linewidth=2, label='φ')
    axes[1].set_ylabel('θ (degrees)', fontsize=11, color='b')
    ax1_twin.set_ylabel('φ (degrees)', fontsize=11, color='r')
    axes[1].set_xlabel('Time (sec)', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('(b) Bending and orientation angles', fontsize=11)
    
    plt.suptitle('Figure 10: FDR Example 2 (Varying Forces)', 
                 fontsize=13, fontweight='bold')
    plt.tight_layout()
    return fig


def plot_idr_example1():
    from simulate import simulate_idr_example1
    from params import L
    import matplotlib.pyplot as plt
    import numpy as np
    
    print("  Running IDR Example 1 simulation...")
    t, theta_d, phi_d, F_req = simulate_idr_example1()
    
    fig = plt.figure(figsize=(13, 6))
    
    # ----------------------------------------------------
    # Subplot a: 3D Workspace (Desired Path)
    # ----------------------------------------------------
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    
    X = np.zeros_like(theta_d)
    Y = np.zeros_like(theta_d)
    Z = np.zeros_like(theta_d)
    
    # محاسبه مختصات فضایی (X, Y, Z) نوک ربات در میلی‌متر
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
    ax1.set_title("(a) Desired path plotted on 2-DOF CDCR's workspace")
    ax1.set_xlabel("X (mm)")
    ax1.set_ylabel("Y (mm)")
    ax1.set_zlabel("Z (mm)")
    
    # تنظیم ابعاد کادر برای اینکه دایره کاملا واضح و متناسب دیده شود
    ax1.set_xlim([-150, 150])
    ax1.set_ylim([-150, 150])
    # Z حدودا روی 793 میلی متر در نوسان است، بنابراین کادر را محدود می‌کنیم:
    ax1.set_zlim([750, 800]) 
    
    # ----------------------------------------------------
    # Subplot b: Temporal evolution of actuation forces
    # ----------------------------------------------------
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(t, F_req[0], 'k-', linewidth=1.5, label='F1')
    ax2.plot(t, F_req[1], 'b--', linewidth=1.5, label='F2')
    ax2.plot(t, F_req[2], 'r-.', linewidth=1.5, label='F3')
    
    ax2.set_title("(b) Temporal evolution of the actuation forces")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Tension (N)")
    ax2.grid(True)
    ax2.legend()
    
    plt.tight_layout()
    return fig


def plot_fdr_example2(t, state, F1_vals, F2_vals, F3_vals):
    theta = state[0, :]
    phi = state[1, :]
    
    # 1. محاسبه مختصات دکارتی نقطه انتهایی (سینماتیک مستقیم - معادله 1)
    X_mm = np.zeros_like(theta)
    Z_mm = np.zeros_like(theta)
    
    for i in range(len(theta)):
        th = theta[i]
        ph = phi[i]
        if abs(th) < 1e-6:
            X_mm[i] = 0.0
            Z_mm[i] = L * 1000.0  # تبدیل متر به میلی‌متر
        else:
            X_mm[i] = (L / th) * (1 - np.cos(th)) * np.cos(ph) * 1000.0
            Z_mm[i] = (L / th) * np.sin(th) * 1000.0

    # 2. رسم دقیقاً مطابق قالب مقاله
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    
    # Subfig 1: Temporal evolution of cable tensions
    ax1.plot(t, F1_vals, 'k-', linewidth=1.5, label='F1')
    ax1.plot(t, F2_vals, 'b--', linewidth=1.5, label='F2')
    ax1.plot(t, F3_vals, 'r-.', linewidth=1.5, label='F3')
    ax1.set_title("Temporal evolution of cable tensions")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Tension (N)")
    ax1.grid(True)
    ax1.legend()
    
    # Subfig 2: Cartesian coordinates
    ax2.plot(X_mm, Z_mm, 'k-', linewidth=2)
    ax2.set_title("Cartesian coordinates of the end-point")
    ax2.set_xlabel("X (mm)")
    ax2.set_ylabel("Z (mm)")
    
    # اعمال دقیق لیمیت‌های درخواستی شما
    ax2.set_xlim([0, 600])
    ax2.set_ylim([380, 830]) 
    
    # نکته: اگر می‌خواهید خط حرکت 5 نیوتنی را ببینید، خط بالا را کامنت کنید 
    # و به جای آن از کد زیر استفاده کنید:
    # ax2.set_ylim([750, 810])
    
    ax2.grid(True)
    plt.tight_layout()
    plt.show()
    return fig

def plot_idr_example2():
    from simulate import simulate_idr_example2
    from params import L
    import matplotlib.pyplot as plt
    import numpy as np
    
    print("  Running IDR Example 2 simulation...")
    t, theta_d, phi_d, F_req = simulate_idr_example2()
    
    fig = plt.figure(figsize=(13, 6))
    
    # ----------------------------------------------------
    # Subplot a: 3D Workspace (Desired Linear Path)
    # ----------------------------------------------------
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
    ax1.set_title("(a) Desired linear path on 2-DOF CDCR's workspace")
    ax1.set_xlabel("X (mm)")
    ax1.set_ylabel("Y (mm)")
    ax1.set_zlabel("Z (mm)")
    
    # تنظیم ابعاد کادر برای مسیر خطی
    ax1.set_xlim([0, 300])
    ax1.set_ylim([0, 300])
    ax1.set_zlim([700, 850])
    
    # ----------------------------------------------------
    # Subplot b: Temporal evolution of actuation forces
    # ----------------------------------------------------
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(t, F_req[0], 'k-', linewidth=1.5, label='F1')
    ax2.plot(t, F_req[1], 'b--', linewidth=1.5, label='F2')
    ax2.plot(t, F_req[2], 'r-.', linewidth=1.5, label='F3')
    
    ax2.set_title("(b) Temporal evolution of the actuation forces")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Tension (N)")
    ax2.grid(True)
    ax2.legend()
    
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
    
    print("Figure 3 (H₃, H₄)...")
    figs['fig3'] = plot_H3_H4_comparison()
    
    print("Figure 8 (Static Equilibrium)...")
    figs['fig8'] = plot_static_equilibrium()
    
    print("Figure 9 (FDR Example 1)...")
    figs['fig9'] = plot_fdr_example1()
    
    print("Figure 10 (FDR Example 2)...")
    # 🔴 تغییر کلیدی برای رفع ارور در این دو خط است:
    t_fdr2, state_fdr2, f1, f2, f3 = simulate_fdr_example2()
    figs['fig10'] = plot_fdr_example2(t_fdr2, state_fdr2, f1, f2, f3)
    
    print("Figure 12 (IDR Example 1)...")
    figs['fig12'] = plot_idr_example1()
    
    print("Figure 13 (IDR Example 2)...")
    figs['fig13'] = plot_idr_example2()
    
    print("Figure 14 (PID Control)...")
    figs['fig14'] = plot_pid_control()
    
    if show:
        plt.show()
    
    print("\n" + "="*70)
    print("✓ All plots generated successfully!")
    print("="*70 + "\n")
    
    return figs


if __name__ == '__main__':
    figs = generate_all_plots(show=True, save=False)
