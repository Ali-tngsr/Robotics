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

import os

def generate_paper_novelty_plots(m_p_load=0.150, show=True, save=False):
    dt = 0.01
    t_final_ctrl = 10.0
    setpoint = 15.53

    print("\n" + "="*60)
    print(f" GENERATING NOVELTY PLOTS FOR PAPER (Payload = {int(m_p_load*1000)}g)")
    print("="*60)

    # ---------------------------------------------------------
    # PART 1: CONTROLLER COMPARISON SIMULATIONS
    # ---------------------------------------------------------
    print("[1/6] Simulating PID (Nominal - 0g)...")
    t_c, state_pid_nom, f_pid_nom = simulate_pid_control(setpoint, t_final_ctrl, dt, m_p=0.0)
    
    print(f"[2/6] Simulating PID (Loaded - {int(m_p_load*1000)}g)...")
    _, state_pid_load, f_pid_load = simulate_pid_control(setpoint, t_final_ctrl, dt, m_p=m_p_load)
    
    print("[3/6] Simulating SMC (Nominal - 0g)...")
    _, state_smc_nom, f_smc_nom = simulate_smc_control(setpoint, t_final_ctrl, dt, m_p=0.0)
    
    print(f"[4/6] Simulating SMC (Loaded - {int(m_p_load*1000)}g)...")
    _, state_smc_load, f_smc_load = simulate_smc_control(setpoint, t_final_ctrl, dt, m_p=m_p_load)

    th_pid_nom = state_pid_nom[0] * 180 / np.pi
    th_pid_load = state_pid_load[0] * 180 / np.pi
    th_smc_nom = state_smc_nom[0] * 180 / np.pi
    th_smc_load = state_smc_load[0] * 180 / np.pi

    ise_pid_nom = np.sum((setpoint - th_pid_nom)**2) * dt
    ise_pid_load = np.sum((setpoint - th_pid_load)**2) * dt
    ise_smc_nom = np.sum((setpoint - th_smc_nom)**2) * dt
    ise_smc_load = np.sum((setpoint - th_smc_load)**2) * dt


    # ---------------------------------------------------------
    # PART 2: DYNAMIC DEGRADATION SIMULATIONS
    # ---------------------------------------------------------
    print(f"\n[5/6] Simulating FDR Workspace Impact (0g vs {int(m_p_load*1000)}g)...")
    t_fdr, state_fdr_base, F1_fdr, F2_fdr, F3_fdr = simulate_fdr_example2(t_final=10.0, m_p=0.0)
    _, state_fdr_load, _, _, _ = simulate_fdr_example2(t_final=10.0, m_p=m_p_load)
    
    print(f"[6/6] Simulating IDR Actuation Cost (0g vs {int(m_p_load*1000)}g)...")
    t_idr, th_d, ph_d, F_idr_base = simulate_idr_example1(m_p=0.0)
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
    #axA.set_ylim([0, 30])
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
    #axB.set_ylim([0, 30])
    axB.grid(True, linestyle=':', alpha=0.7)
    axB.legend(loc='lower right')
    figB.tight_layout()

    # ==========================================
    # FIGURE C: Control Effort (Chattering Check)
    # ==========================================
    figC, axC = plt.subplots(figsize=(8, 5))
    axC.plot(t_c, f_smc_load[0], 'b-', linewidth=1.5, label='SMC Actuation Force')
    axC.plot(t_c, f_pid_load[0], 'r--', linewidth=1.5, alpha=0.7, label='PID Actuation Force')
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
    # FIGURE E: FDR Open-Loop Degradation (1x2 Subplot like Paper Fig 10)
    # ==========================================
    def calc_cartesian_2d(theta, phi):
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

    X_base, Z_base = calc_cartesian_2d(state_fdr_base[0], state_fdr_base[1])
    X_load, Z_load = calc_cartesian_2d(state_fdr_load[0], state_fdr_load[1])
    
    figE = plt.figure(figsize=(12, 5))
    
    # Subplot E(a): Forces
    axE1 = figE.add_subplot(1, 2, 1)
    axE1.plot(t_fdr, F1_fdr, 'k-', linewidth=1.5, label='F1')
    axE1.plot(t_fdr, F2_fdr, 'b--', linewidth=1.5, label='F2')
    axE1.plot(t_fdr, F3_fdr, 'r-.', linewidth=1.5, label='F3')
    axE1.set_title("(a) Temporal evolution of cable tensions")
    axE1.set_xlabel("Time (s)")
    axE1.set_ylabel("Tension (N)")
    axE1.grid(True, linestyle=':', alpha=0.7)
    axE1.legend()

    # Subplot E(b): Cartesian Path Sagging
    axE2 = figE.add_subplot(1, 2, 2)
    axE2.plot(X_base, Z_base, 'k-', linewidth=2, label='Baseline Path (0g Payload)')
    axE2.plot(X_load, Z_load, 'r--', linewidth=2, label=f'Degraded Path ({int(m_p_load*1000)}g Payload)')
    axE2.set_title("(b) Cartesian coordinates of the end-point")
    axE2.set_xlabel("X (mm)")
    axE2.set_ylabel("Z (mm)")
    axE2.set_xlim([0, 600])
    axE2.set_ylim([350, 850])
    axE2.grid(True, linestyle=':', alpha=0.7)
    axE2.legend(loc='lower left')
    
    figE.suptitle("Fig. E: Open-Loop Workspace Sagging due to Payload (FDR)", fontweight='bold', fontsize=13)
    figE.tight_layout()

    # ==========================================
    # FIGURE F: IDR Actuation Cost (1x2 Subplot like Paper Fig 12)
    # ==========================================
    def calc_cartesian_3d(theta, phi):
        X = np.zeros_like(theta)
        Y = np.zeros_like(theta)
        Z = np.zeros_like(theta)
        for i in range(len(theta)):
            th, ph = theta[i], phi[i]
            if abs(th) < 1e-6:
                X[i], Y[i], Z[i] = 0.0, 0.0, L * 1000.0
            else:
                X[i] = (L / th) * (1 - np.cos(th)) * np.cos(ph) * 1000.0
                Y[i] = (L / th) * (1 - np.cos(th)) * np.sin(ph) * 1000.0
                Z[i] = (L / th) * np.sin(th) * 1000.0
        return X, Y, Z

    X_idr, Y_idr, Z_idr = calc_cartesian_3d(th_d, ph_d)
    
    figF = plt.figure(figsize=(13, 5))
    
    # Subplot F(a): 3D Path
    axF1 = figF.add_subplot(1, 2, 1, projection='3d')
    axF1.plot(X_idr, Y_idr, Z_idr, 'b-', linewidth=2.5)
    axF1.set_title("(a) Desired path on CDCR workspace")
    axF1.set_xlabel("X (mm)")
    axF1.set_ylabel("Y (mm)")
    axF1.set_zlabel("Z (mm)")
    axF1.set_xlim([-150, 150])
    axF1.set_ylim([-150, 150])
    axF1.set_zlim([750, 800])
    
    # Subplot F(b): Force Comparison
    axF2 = figF.add_subplot(1, 2, 2)
    
    axF2.plot(t_idr, F_idr_base[0], 'k--', linewidth=1.5, label='F1 (Baseline)')
    axF2.plot(t_idr, F_idr_load[0], 'k-',  linewidth=2.0, label='F1 (Loaded)')
    
    axF2.plot(t_idr, F_idr_base[1], 'b--', linewidth=1.5, label='F2 (Baseline)')
    axF2.plot(t_idr, F_idr_load[1], 'b-',  linewidth=2.0, label='F2 (Loaded)')
    
    axF2.plot(t_idr, F_idr_base[2], 'r--', linewidth=1.5, label='F3 (Baseline)')
    axF2.plot(t_idr, F_idr_load[2], 'r-',  linewidth=2.0, label='F3 (Loaded)')

    axF2.set_title("(b) Temporal evolution of the actuation forces")
    axF2.set_xlabel("Time (s)")
    axF2.set_ylabel("Tension (N)")
    axF2.grid(True, linestyle=':', alpha=0.7)
    
    # Legend at the bottom
    axF2.legend(ncol=3, loc='upper center', bbox_to_anchor=(0.5, -0.15), fontsize=9)
    
    figF.suptitle("Fig. F: Additional Actuation Effort for 3D Trajectory Tracking (IDR)", fontweight='bold', fontsize=13)
    figF.subplots_adjust(bottom=0.2) # Make room for legend
    figF.tight_layout()

    # ==========================================
    # Collect Figures
    # ==========================================
    figs = {
        'figA_pid_failure': figA,
        'figB_smc_robustness': figB,
        'figC_control_effort': figC,
        'figD_ise_comparison': figD,
        'figE_fdr_workspace_sagging': figE,
        'figF_idr_actuation_cost': figF,
    }

    # ==========================================
    # Save Figures
    # ==========================================
    if save:
        print("\nSaving high-quality plots...")

        save_dir = os.path.join(
            "figures",
            f"payload_{int(m_p_load*1000)}g"
        )

        os.makedirs(save_dir, exist_ok=True)

        for fig_name, fig in figs.items():
            if fig is not None:
                filepath = os.path.join(save_dir, f"{fig_name}.png")

                fig.savefig(
                    filepath,
                    dpi=300,
                    bbox_inches='tight',
                    format='png'
                )

                print(f"  -> Saved: {filepath}")

    # ==========================================
    # Show Figures
    # ==========================================
    if show:
        plt.show()

    print("\n" + "="*60)
    print("✓ All 6 Novelty Figures generated successfully!")
    print("="*60)

    return figs

if __name__ == '__main__':

    generate_paper_novelty_plots(
        m_p_load=0.050,
        show=True,
        save=True
    )