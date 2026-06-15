"""
PHASE 5: SIMULATIONS
Five scenarios from Amouri et al. 2020:
1. Static equilibrium (Fig. 8)
2. Forward dynamics response Example 1 (Fig. 9)
3. Forward dynamics response Example 2 (Fig. 10)
4. Inverse dynamics response Example 1 (Fig. 12)
5. Inverse dynamics response Example 2 (Fig. 13)
[Bonus] PID control (Fig. 14)

(Refactored: Centralized Imports, Modular PID, and Dependency Injection for m_p)
"""
import numpy as np
from scipy.integrate import solve_ivp

# وارد کردن توابع دینامیکی
from dynamics import state_derivative, inverse_dynamics_3cables

# وارد کردن کنترلر از فایل مستقل
from pid_controller import SimplePIDController


# ─────────────────────────────────────────────────────────────────────────────
# 1. STATIC EQUILIBRIUM (Fig. 8)
# ─────────────────────────────────────────────────────────────────────────────
def simulate_static_equilibrium(t_final=40.0, num_points=1000, m_p=0.0):
    """
    Static equilibrium with no cable actuation.
    System should oscillate and stabilize around θ=0 (gravity negligible without payload).
    """
    state0 = [np.pi/4, 0.0, 0.0, 0.0]
    t_span = (0, t_final)
    t_eval = np.linspace(0, t_final, num_points)
    
    def no_force(t):
        return np.array([0.0, 0.0])
    
    # توجه: m_p به عنوان آرگومان به state_derivative ارسال می‌شود
    sol = solve_ivp(state_derivative, t_span, state0, t_eval=t_eval,
                    args=(no_force, m_p), method='RK45', dense_output=False)
    
    return sol.t, sol.y  # [θ, φ, θ̇, φ̇]


# ─────────────────────────────────────────────────────────────────────────────
# 2. FORWARD DYNAMICS RESPONSE Example 1 (Fig. 9)
# ─────────────────────────────────────────────────────────────────────────────
def simulate_fdr_example1(t_final=40.0, num_points=1000, F1=5.0, m_p=0.0):
    """
    5N constant tension on cable 1.
    """
    state0 = [1e-8, 0.0, 0.0, 0.0]  # Avoid exact zero
    t_span = (0, t_final)
    t_eval = np.linspace(0, t_final, num_points)
    
    def constant_force1(t):
        return np.array([F1, 0.0])
    
    sol = solve_ivp(state_derivative, t_span, state0, t_eval=t_eval,
                    args=(constant_force1, m_p), method='RK45')
    
    return sol.t, sol.y


# ─────────────────────────────────────────────────────────────────────────────
# 3. FORWARD DYNAMICS RESPONSE Example 2 (Fig. 10)
# ─────────────────────────────────────────────────────────────────────────────
def simulate_fdr_example2(t_final=10.0, num_points=1000, m_p=0.0):
    state0 = [1e-8, 0.0, 0.0, 0.0]
    t_span = (0, t_final)
    t_eval = np.linspace(0, t_final, num_points)
    
    def force_func(t):
        F1 = 3.5 * t
        F2 = 0.0
        return np.array([F1, F2])
    
    sol = solve_ivp(state_derivative, t_span, state0, t_eval=t_eval,
                    args=(force_func, m_p), method='RK45')
    
    # محاسبه مقادیر نیروها در طول زمان برای ارسال به بخش رسم نمودار
    F1_vals = 3.5 * sol.t
    F2_vals = np.zeros_like(sol.t)
    F3_vals = np.zeros_like(sol.t)
    
    return sol.t, sol.y, F1_vals, F2_vals, F3_vals


# ─────────────────────────────────────────────────────────────────────────────
# 4. INVERSE DYNAMICS Example 1 (Fig. 12)
# ─────────────────────────────────────────────────────────────────────────────
def simulate_idr_example1(m_p=0.0):
    """
    Track circular trajectory: θ = π/12, φ = (π/5)·t
    """
    t_start, t_end = 0.0, 10.0
    num_points = 501
    t = np.linspace(t_start, t_end, num_points)
    
    # مسیر مطلوب
    theta_d = np.ones_like(t) * (np.pi / 12)
    phi_d = (np.pi / 5) * t
    dtheta_d = np.zeros_like(t)
    ddtheta_d = np.zeros_like(t)
    dphi_d = np.ones_like(t) * (np.pi / 5)
    ddphi_d = np.zeros_like(t)
    
    F_array = np.zeros((3, num_points))
    
    for i in range(num_points):
        F = inverse_dynamics_3cables(theta_d[i], phi_d[i],
                                     dtheta_d[i], dphi_d[i],
                                     ddtheta_d[i], ddphi_d[i],
                                     m_p=m_p)  # تزریق m_p
        F_array[:, i] = F
        
    return t, theta_d, phi_d, F_array


# ─────────────────────────────────────────────────────────────────────────────
# 5. INVERSE DYNAMICS Example 2 (Fig. 13)
# ─────────────────────────────────────────────────────────────────────────────
def simulate_idr_example2(m_p=0.0):
    """
    Track trajectory: θ = (π/4)·t, φ = π/6
    """
    # زمان را به 1 ثانیه کاهش دادیم تا زاویه نهایی 45 درجه (pi/4) شود
    t_start, t_end = 0.0, 1.0  
    num_points = 501
    t = np.linspace(t_start, t_end, num_points)
    
    # مسیر مطلوب خطی
    theta_d = (np.pi / 4) * t
    phi_d = np.ones_like(t) * (np.pi / 6)
    dtheta_d = np.ones_like(t) * (np.pi / 4)
    ddtheta_d = np.zeros_like(t)
    dphi_d = np.zeros_like(t)
    ddphi_d = np.zeros_like(t)
    
    F_array = np.zeros((3, num_points))
    
    for i in range(num_points):
        F = inverse_dynamics_3cables(theta_d[i], phi_d[i],
                                     dtheta_d[i], dphi_d[i],
                                     ddtheta_d[i], ddphi_d[i],
                                     m_p=m_p)  # تزریق m_p
        F_array[:, i] = F
    
    return t, theta_d, phi_d, F_array


# ─────────────────────────────────────────────────────────────────────────────
# BONUS: PID CONTROL (Fig. 14)
# ─────────────────────────────────────────────────────────────────────────────
def simulate_pid_control(setpoint=15.53, t_final=5.0, dt=0.01, m_p=0.0):
    """
    PID control to reach bending angle setpoint (Fig. 14).
    Paper parameters: Kp=2.8, Ki=0.004, Kd=0.38
    """
    setpoint_rad = setpoint * np.pi / 180.0
    pid = SimplePIDController(Kp=2.8, Ki=0.004, Kd=0.38)
    
    t = np.arange(0, t_final, dt)
    state = np.array([1e-8, 0.0, 0.0, 0.0])  # [θ, φ, θ̇, φ̇]
    
    history = np.zeros((4, len(t)))
    force_history = np.zeros((3, len(t)))
    
    # تکنیک پیش‌خور برای غلبه بر فنریت در نقطه کار
    F_feedforward = 5.0 
    
    for i, current_t in enumerate(t):
        history[:, i] = state
        
        # محاسبه خطا به رادیان
        error = setpoint_rad - state[0]
        
        # خروجی PID فقط وظیفه اصلاح لرزش‌ها را دارد
        pid_action = pid.update(error, dt)
        
        # نیروی نهایی = نیروی تعادل + خروجی کنترلر
        F1 = max(0.0, F_feedforward + pid_action) 
        F2 = 0.0
        F3 = 0.0
        
        force_history[:, i] = [F1, F2, F3]
        
        def force_func(t_inner):
            return np.array([F1, F2])
        
        # حل یک گام زمانی با در نظر گرفتن m_p
        sol = solve_ivp(state_derivative, [current_t, current_t + dt],
                       state, args=(force_func, m_p), method='RK45')
        state = sol.y[:, -1]
        
    return t, history, force_history
