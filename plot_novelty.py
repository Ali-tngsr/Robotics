"""
NOVELTY PLOTS FOR THE PAPER (Extension Results)
Generates all 6 core figures proving the superiority of the proposed 
Dynamic Model and SMC over the baseline PID under payload disturbances.
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

# تنظیمات فونت و ظاهر برای مقالات ژورنالی
mpl.rcParams['font.family'] = 'serif'
mpl.rcParams['axes.titlesize'] = 12
mpl.rcParams['axes.labelsize'] = 11
mpl.rcParams['legend.fontsize'] = 10

# وارد کردن پارامترها و شبیه‌سازها
from params import L
from simulate import (simulate_pid_control, simulate_smc_control, 
                      simulate_fdr_example2, simulate_idr_example1)

def generate_paper_novelty_plots(m_p_load=0.150):
    dt = 0.01
    t_final_ctrl = 5.0
    setpoint = 15.53
    
    print("\n" + "="*60)
    print(f" GENERATING NOVELTY PLOTS FOR PAPER (Payload = {int(m_p_load*1000)}g)")
    print("="*60)

    # ---------------------------------------------------------
    # PART 1: CONTROLLER COMPARISON SIMULATIONS (Figs A, B, C, D)
    # ---------------------------------------------------------
    print("[1/6] Simulating PID (Nominal - 0g)...")
    t_c, state_pid_nom, f_pid_nom = simulate_pid_control(setpoint, t_final_ctrl, dt, m_p=0.0)
    
    print(f"[2/6] Simulating PID (Loaded - {int(m_p_load*1000)}g)...")
    _, state_pid_load, f_pid_load = simulate_pid_control(setpoint, t_final_ctrl, dt, m_p=m_p_load)
    
    print("[3/6] Simulating SMC (Nominal - 0g)...")
    _, state_smc_nom, f_smc_nom = simulate_smc_control(setpoint, t_final_ctrl, dt, m_p=0.0)
    
    print(f"[4/6] Simulating SMC (Loaded - {int(m_p_load*1000)}g)...")
    _, state_smc_load, f_smc_load = simulate_smc_control(setpoint, t_final_ctrl, dt, m_p=m_p_load)

    # تبدیل زوایا به درجه
    th_pid_nom = state_pid_nom[0] * 180 / np.pi
    th_pid_load = state_pid_load[0] * 180 / np.pi
    th_smc_nom = state_smc_nom[0] * 180 / np.pi
    th_smc_load = state_smc_load[0] * 180 / np.pi

    # محاسبه شاخص ریاضی ISE (Integral Squared Error)
    ise_pid_nom = np.sum((setpoint - th_pid_nom)**2) * dt
    ise_pid_load = np.sum((setpoint - th_pid_load)**2) * dt
    ise_smc_nom = np.sum((setpoint - th_smc_nom)**2) * dt
    ise_smc_load = np.sum((setpoint - th_smc_load)**2) * dt


    # ---------------------------------------------------------
    # PART 2: DYNAMIC DEGRADATION SIMULATIONS (Figs E, F)
    # ---------------------------------------------------------
    print(f"\n[5/6] Simulating FDR Workspace Impact (0g vs {int(m_p_load*1000)}g)...")
    t_fdr, state_fdr_base, _, _, _ = simulate_fdr_example2(t_final=10.0, m_p=0.0)
    _, state_fdr_load, _, _, _ = simulate_fdr_example2(t_final=10.0, m_p=m_p_load)
    
    print(f"[6/6] Simulating IDR Actuation Cost (0g vs {int(m_p_load*1000)}g)...")
    t_idr, _, _, F_idr_base = simulate_idr_example1(m_p=0.0)
    _, _, _, F_idr_load = simulate_idr_example1(m_p=m_p_load)

    print("\nGenerating Figures...")

    # ==========================================
    # FIGURE A: Problem Statement (PID Failure)
    # ==========================================
    figA, axA = plt.subplots(figsize=(8, 5))
    axA.plot(t_c, th_pid_nom, 'g-', linewidth=2, label='PID Controller (Nominal: 0g)')
    axA.plot(t_c, th_pid_load, 'r--', linewidth=2.5, label=f'PID Controller (Loaded: {int(m_p_load*1000)}g)')
    axA.axhline(setpoint, color='k', linestyle=':', linewidth=1.5, label='Target Reference (15.53°)')
    axA.set_title("Fig. A: Problem Statement - Degradation of PID under Payload", fontweight='bold')
    axA.set_xlabel("Time (s)")
    axA.set_ylabel(r"Bending Angle $\theta$ (degrees)")
    axA.set_ylim([0, 18])
    axA.grid(True, linestyle=':', alpha=0.7)
    axA.legend(loc='lower right')
    figA.tight_layout()

    # ==========================================
    # FIGURE B: The Solution (SMC Robustness)
    # ==========================================
    figB, axB = plt.subplots(figsize=(8, 5))
    axB.plot(t_c, th_pid_load, 'r--', linewidth=2, alpha=0.6, label=f'PID Controller (Loaded)')
    axB.plot(t_c, th_smc_load, 'b-', linewidth=2.5, label=f'SMC Controller (Loaded)')
    axB.axhline(setpoint, color='k', linestyle=':', linewidth=1.5, label='Target Reference (15.53°)')
    axB.set_title("Fig. B: Robust Tracking - SMC vs PID under Payload Disturbance", fontweight='bold')
    axB.set_xlabel("Time (s)")
    axB.set_ylabel(r"Bending Angle $\theta$ (degrees)")
    axB.set_ylim([0, 18])
    axB.grid(True, linestyle=':', alpha=0.7)
    axB.legend(loc='lower right')
    figB.tight_layout()

    # ==========================================
    # FIGURE C: Control Effort (Chattering Check)
    # ==========================================
    figC, axC = plt.subplots(figsize=(8, 5))
    axC.plot(t_c, f_pid_load[0], 'r--', linewidth=1.5, alpha=0.7, label='PID Actuation Force')
    axC.plot(t_c, f_smc_load[0], 'b-', linewidth=1.5, label='SMC Actuation Force')
    axC.set_title("Fig. C: Control Effort & Chattering Elimination Analysis", fontweight='bold')
    axC.set_xlabel("Time (s)")
    axC.set_ylabel(r"Cable Tension $F_1$ (N)")
    axC.grid(True, linestyle=':', alpha=0.7)
    axC.legend(loc='upper right')
    figC.tight_layout()

    # ==========================================
    # FIGURE D: Quantitative Bar Chart (ISE)
    # ==========================================
    figD, axD = plt.subplots(figsize=(8, 5))
    labels = ['PID\n(Nominal)', 'SMC\n(Nominal)', 'PID\n(Loaded)', 'SMC\n(Loaded)']
    ise_values = [ise_pid_nom, ise_smc_nom, ise_pid_load, ise_smc_load]
    colors = ['#8fce00', '#5b9bd5', '#ff6b6b', '#1f4e79']

    bars = axD.bar(labels, ise_values, color=colors, edgecolor='black', width=0.6)
    axD.set_title("Fig. D: Quantitative Robustness Analysis (Integral Squared Error)", fontweight='bold')
    axD.set_ylabel(r"Tracking Error ISE ($deg^2 \cdot s$)")
    axD.grid(axis='y', linestyle=':', alpha=0.7)

    for bar in bars:
        yval = bar.get_height()
        axD.text(bar.get_x() + bar.get_width()/2, yval + (max(ise_values)*0.02),
                 f'{yval:.1f}', ha='center', va='bottom', fontweight='bold')
    figD.tight_layout()

    # ==========================================
    # FIGURE E: FDR Open-Loop Degradation (Sagging)
    # ==========================================
    def calc_cartesian(theta, phi):
        X = np.zeros_like(theta)
        Z = np.zeros_like(theta)
        for i in range(len(theta)):
            th, ph = theta[i], phi[i]
            if abs(th) < 1e-6:
                X[i], Z[i] = 0.0, L * 1000.0
            else:
                X[i] = (L / th) * (1 - np.cos(th)) * np.cos(ph) * 1000.0
                Z[i] = (L / th) * np.sin(th) * 1000.0
        return X, Z

    X_base, Z_base = calc_cartesian(state_fdr_base[0], state_fdr_base[1])
    X_load, Z_load = calc_cartesian(state_fdr_load[0], state_fdr_load[1])
    
    figE, axE = plt.subplots(figsize=(8, 6))
    axE.plot(X_base, Z_base, 'k-', linewidth=2.5, label='Baseline Path (0g Payload)')
    axE.plot(X_load, Z_load, 'r--', linewidth=2.5, label=f'Degraded Path ({int(m_p_load*1000)}g Payload)')
    
    # رسم خطوط اتصال نقطه به نقطه برای نشان دادن افتادگی
    for i in range(0, len(X_base), 150):
        axE.plot([X_base[i], X_load[i]], [Z_base[i], Z_load[i]], 'gray', linestyle=':', alpha=0.6)
        
    axE.set_title("Fig. E: Open-Loop Workspace Sagging due to Payload (FDR)", fontweight='bold')
    axE.set_xlabel("Cartesian X (mm)")
    axE.set_ylabel("Cartesian Z (mm)")
    axE.set_xlim([0, 600])
    axE.set_ylim([350, 850])
    axE.grid(True, linestyle=':', alpha=0.7)
    axE.legend(loc='lower left')
    figE.tight_layout()

    # ==========================================
    # FIGURE F: IDR Actuation Cost
    # ==========================================
    figF, axesF = plt.subplots(3, 1, figsize=(9, 10), sharex=True)
    cable_colors = ['k', 'b', 'g']
    cable_names = ['Cable 1', 'Cable 2', 'Cable 3']
    
    for i in range(3):
        axesF[i].plot(t_idr, F_idr_base[i], color=cable_colors[i], linestyle='-', linewidth=2, 
                     label=f'{cable_names[i]} (Baseline - 0g)')
        axesF[i].plot(t_idr, F_idr_load[i], color='r', linestyle='--', linewidth=2, 
                     label=f'{cable_names[i]} (Loaded - {int(m_p_load*1000)}g)')
        
        # هاشور زدن اختلاف برای نشان دادن هزینه تلاش اضافی موتورها
        axesF[i].fill_between(t_idr, F_idr_base[i], F_idr_load[i], color='red', alpha=0.1)
        
        axesF[i].set_ylabel("Tension (N)")
        axesF[i].grid(True, linestyle=':', alpha=0.7)
        axesF[i].legend(loc='upper right', fontsize=9)
    
    axesF[0].set_title("Fig. F: Additional Actuation Effort for 3D Trajectory Tracking (IDR)", fontweight='bold')
    axesF[2].set_xlabel("Time (s)")
    figF.tight_layout()

    print("\n✓ All 6 Novelty Figures generated successfully!")
    print("Close the plot windows to proceed.")
    plt.show()

if __name__ == '__main__':
    generate_paper_novelty_plots(m_p_load=0.150)
