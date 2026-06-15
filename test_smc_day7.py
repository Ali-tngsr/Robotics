import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import warnings
warnings.filterwarnings("ignore")

import params
from dynamics import mass_matrix, coriolis_matrix, stiffness_matrix, gravity_vector
from smc_controller import SMCController

# تنظیم جرم بار (۱۰۰ گرم)
params.m_p = 0.100  
# فراخوانی کنترلر با پارامترهای جدید شامل K1 و K2
smc = SMCController(lambda_1=8.0, lambda_2=8.0, k_1=5.0, k_2=5.0)

def desired_trajectory(t):
    theta_d = np.pi / 12
    phi_d = (np.pi / 5) * t
    return np.array([theta_d, phi_d]), np.array([0.0, np.pi/5]), np.array([0.0, 0.0])

def force_matrix_3cables(theta, phi):
    r = params.r_cab
    gam1, gam2, gam3 = 0.0, 2*np.pi/3, 4*np.pi/3
    
    D11 = r * np.cos(gam1 - phi)
    D12 = r * np.cos(gam2 - phi)
    D13 = r * np.cos(gam3 - phi)
    
    D21 = r * theta * np.sin(gam1 - phi)
    D22 = r * theta * np.sin(gam2 - phi)
    D23 = r * theta * np.sin(gam3 - phi)
    
    return np.array([[D11, D12, D13],
                     [D21, D22, D23]])

def closed_loop_dynamics(t, state):
    theta, phi, theta_dot, phi_dot = state
    q = np.array([theta, phi])
    q_dot = np.array([theta_dot, phi_dot])
    vel = np.array([theta_dot**2, theta_dot*phi_dot, phi_dot**2])
    
    q_d, q_d_dot, q_d_ddot = desired_trajectory(t)
    
    M = mass_matrix(theta)
    C = coriolis_matrix(theta)
    K = stiffness_matrix()
    G = gravity_vector(theta)
    
    D3 = force_matrix_3cables(theta, phi)
    C_v = C @ vel
    
    # 🌟 اینجا تابع جدید get_control فراخوانی می‌شود
    F_cmd, s, e = smc.get_control(q, q_dot, q_d, q_d_dot, q_d_ddot, M, C_v, K, G, D3)
    
    # 🌟 استفاده از نیروی کامل (F_cmd) به جای نیروی معادل تنها
    q_ddot = np.linalg.solve(M, D3 @ F_cmd - C_v - K @ q - G)
    return [theta_dot, phi_dot, q_ddot[0], q_ddot[1]]

print("Simulating Full SMC Control (Day 8) on Circular Trajectory...")
t_eval = np.linspace(0, 10, 500)
state0 = [1e-4, 0.0, 0.0, 0.0] 
sol = solve_ivp(closed_loop_dynamics, [0, 10], state0, t_eval=t_eval, method='LSODA')

theta_actual = sol.y[0, :]
theta_desired = np.ones_like(t_eval) * (np.pi / 12)

plt.figure(figsize=(8, 4))
plt.plot(t_eval, np.degrees(theta_actual), label='Actual Theta', color='blue')
plt.plot(t_eval, np.degrees(theta_desired), '--', label='Desired Theta (15 deg)', color='red')
plt.title("Day 8: Full SMC Tracking (3 Cables - with Switching Control)")
plt.xlabel("Time (s)")
plt.ylabel("Theta (degrees)")
plt.legend()
plt.grid(True)
plt.show()