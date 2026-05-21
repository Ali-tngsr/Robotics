import numpy as np
import matplotlib.pyplot as plt
from simulate import run_static_equilibrium, run_fdr_example_1, run_idr_example_1, run_pid_control
from simulate import run_smc_tracking_with_payload, run_pid_tracking_with_payload

def plot_controller_comparison():
    print("   -> Simulating SMC (Robust Tracking)...")
    t_smc, smc_hist, desired_smc, smc_forces, smc_errors = run_smc_tracking_with_payload()
    
    print("   -> Simulating PID (Classical Control)...")
    t_pid, pid_hist, desired_pid, pid_errors = run_pid_tracking_with_payload()
    
    smc_theta = smc_hist[0] * (180 / np.pi)
    pid_theta = pid_hist[0] * (180 / np.pi)
    theta_d = desired_smc[0] * (180 / np.pi)
    
    smc_err_theta = smc_errors[0] * (180 / np.pi)
    pid_err_theta = pid_errors[0] * (180 / np.pi)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=False)
    
    ax1.plot(t_smc, theta_d, 'k--', linewidth=2, label='Desired Path')
    ax1.plot(t_smc, smc_theta, 'b-', linewidth=2, label='SMC (Robust)')
    ax1.plot(t_pid, pid_theta, 'r-', linewidth=2, alpha=0.7, label='PID (Failing)')
    ax1.set_ylabel('Bending Angle Theta (°)')
    ax1.set_title('Robustness Benchmark: Tracking under 50g Unknown Payload')
    ax1.legend(loc='lower left')
    ax1.grid(True)
    
    ax2.axhline(0, color='k', linestyle='--', linewidth=2)
    ax2.plot(t_smc, smc_err_theta, 'b-', linewidth=2, label='SMC Error')
    ax2.plot(t_pid, pid_err_theta, 'r-', linewidth=2, alpha=0.7, label='PID Error')
    ax2.set_ylabel('Tracking Error (°)')
    ax2.set_xlabel('Time (sec)')
    ax2.legend(loc='lower left')
    ax2.grid(True)
    
    plt.tight_layout()
    plt.show()
# Update your main block:
def plot_smc_performance():
    """
    Plots the performance of the SMC tracking a spatial trajectory under payload uncertainty.
    """
    t, history, desired, forces, errors = run_smc_tracking_with_payload()
    
    theta_actual = history[0] * (180 / np.pi)
    phi_actual = history[1] * (180 / np.pi)
    
    theta_desired = desired[0] * (180 / np.pi)
    phi_desired = desired[1] * (180 / np.pi)
    
    error_theta = errors[0] * (180 / np.pi)
    error_phi = errors[1] * (180 / np.pi)
    
    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    
    # Plot 1: Trajectory Tracking
    axs[0].plot(t, theta_desired, 'b--', label='Theta Desired')
    axs[0].plot(t, theta_actual, 'b-', label='Theta Actual')
    axs[0].plot(t, phi_desired, 'r--', label='Phi Desired')
    axs[0].plot(t, phi_actual, 'r-', label='Phi Actual')
    axs[0].set_ylabel('Angles (°)')
    axs[0].set_title('SMC Trajectory Tracking with Unknown Payload (True: 50g, Est: 30g)')
    axs[0].legend(loc='upper right')
    axs[0].grid(True)
    
    # Plot 2: Tracking Error
    axs[1].plot(t, error_theta, 'b-', label='Theta Error')
    axs[1].plot(t, error_phi, 'r-', label='Phi Error')
    axs[1].axhline(0, color='k', linestyle='--', alpha=0.5)
    axs[1].set_ylabel('Error (°)')
    axs[1].legend(loc='upper right')
    axs[1].grid(True)
    
    # Plot 3: Actuation Forces
    axs[2].plot(t, forces[0], 'g-', label='F1 (Cable 1)')
    axs[2].plot(t, forces[1], 'm-', label='F2 (Cable 2)')
    axs[2].set_ylabel('Force (N)')
    axs[2].set_xlabel('Time (sec)')
    axs[2].legend(loc='upper right')
    axs[2].grid(True)
    
    plt.tight_layout()
    plt.show()
    
def plot_static_equilibrium():
    t, y = run_static_equilibrium()
    theta_deg = y[0] * (180 / np.pi)
    phi_deg = y[1] * (180 / np.pi)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 4), sharex=True)
    ax1.plot(t, theta_deg, 'b-')
    ax1.set_ylabel('Theta (°)')
    ax1.grid(True)
    ax1.set_ylim(-50, 50)
    
    ax2.plot(t, phi_deg, 'r-')
    ax2.set_ylabel('Phi (°)')
    ax2.set_xlabel('Time (sec)')
    ax2.grid(True)
    ax2.set_ylim(-1, 1)
    
    plt.suptitle('Base Model Validation: Static Equilibrium (0g Payload)')
    plt.tight_layout()
    plt.show()
    

def plot_fdr_example_1():
    t, y = run_fdr_example_1()
    theta_deg = y[0] * (180 / np.pi)
    phi_deg = y[1] * (180 / np.pi)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 4), sharex=True)
    ax1.plot(t, theta_deg, 'b-')
    ax1.set_ylabel('Theta (°)')
    ax1.grid(True)
    ax1.axhline(15.53, color='k', linestyle='--', alpha=0.5) 
    
    ax2.plot(t, phi_deg, 'r-')
    ax2.set_ylabel('Phi (°)')
    ax2.set_xlabel('Time (sec)')
    ax2.grid(True)
    
    plt.suptitle('Base Model Validation: 5N Step Response (0g Payload)')
    plt.tight_layout()
    plt.show()

def plot_pid_control():
    """ Replicates Fig. 14: Dynamic responses with PID controller[cite: 707]. """
    t, history, force_history = run_pid_control()
    
    theta_deg = history[0] * (180 / np.pi)
    phi_deg = history[1] * (180 / np.pi)
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
    
    ax1.plot(t, theta_deg, 'b-', linewidth=2)
    ax1.axhline(15.53, color='g', linestyle='-')
    ax1.set_ylabel('theta (°)')
    ax1.grid(True)
    ax1.set_ylim(0, 30)
    
    ax2.plot(t, phi_deg, 'r-', linewidth=2)
    ax2.set_ylabel('phi (°)')
    ax2.grid(True)
    
    ax3.plot(t, force_history, 'k-', linewidth=2)
    ax3.axhline(5.0, color='g', linestyle='-')
    ax3.set_ylabel('Force F1 (N)')
    ax3.set_xlabel('Time (sec)')
    ax3.grid(True)
    ax3.set_ylim(0, 10)
    
    plt.suptitle('Fig. 14: Dynamic responses for bending and orientation angles with PID')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    print("Running Static Equilibrium...")
    plot_static_equilibrium()
    
    print("Running Forward Dynamics Example 1...")
    plot_fdr_example_1()
    
    print("Running PID Control Simulation...")
    plot_pid_control()
    
    print("Running SMC Tracking Simulation...")
    plot_smc_performance()
    
    plot_controller_comparison()
