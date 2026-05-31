"""
PHASE 5: SIMULATIONS
Five scenarios from Amouri et al. 2020:
1. Static equilibrium (Fig. 8)
2. Forward dynamics response Example 1 (Fig. 9)
3. Forward dynamics response Example 2 (Fig. 10)
4. Inverse dynamics response Example 1 (Fig. 12)
5. Inverse dynamics response Example 2 (Fig. 13)
[Bonus] PID control (Fig. 14)
"""
import numpy as np
from scipy.integrate import solve_ivp
from dynamics import state_derivative, inverse_dynamics


# ─────────────────────────────────────────────────────────────────────────────
# 1. STATIC EQUILIBRIUM (Fig. 8)
# ─────────────────────────────────────────────────────────────────────────────

def simulate_static_equilibrium(t_final=40.0, num_points=1000):
    """
    Static equilibrium with no cable actuation.
    Initial: θ = π/4, φ = 0, θ̇ = 0, φ̇ = 0
    
    System should oscillate and stabilize around θ=0 (gravity negligible).
    Paper: stabilization ~37.68 sec (Fig. 8).
    """
    state0 = [np.pi/4, 0.0, 0.0, 0.0]
    t_span = (0, t_final)
    t_eval = np.linspace(0, t_final, num_points)
    
    def no_force(t):
        return np.array([0.0, 0.0])
    
    sol = solve_ivp(state_derivative, t_span, state0, t_eval=t_eval,
                    args=(no_force,), method='RK45', dense_output=False)
    
    return sol.t, sol.y  # [θ, φ, θ̇, φ̇]


# ─────────────────────────────────────────────────────────────────────────────
# 2. FORWARD DYNAMICS RESPONSE Example 1 (Fig. 9)
# ─────────────────────────────────────────────────────────────────────────────

def simulate_fdr_example1(t_final=40.0, num_points=1000, F1=5.0):
    """
    5N constant tension on cable 1.
    Initial: θ → 0⁺, φ = 0, θ̇ = 0, φ̇ = 0
    
    Paper: θ → 15.53°, φ stays ~0 (Fig. 9).
    """
    state0 = [1e-8, 0.0, 0.0, 0.0]  # Avoid exact zero
    t_span = (0, t_final)
    t_eval = np.linspace(0, t_final, num_points)
    
    def constant_force1(t):
        return np.array([F1, 0.0])
    
    sol = solve_ivp(state_derivative, t_span, state0, t_eval=t_eval,
                    args=(constant_force1,), method='RK45')
    
    return sol.t, sol.y


# ─────────────────────────────────────────────────────────────────────────────
# 3. FORWARD DYNAMICS RESPONSE Example 2 (Fig. 10)
# ─────────────────────────────────────────────────────────────────────────────

def simulate_fdr_example2(t_final=10.0, num_points=1000):
    from scipy.integrate import solve_ivp
    import numpy as np
    from dynamics import state_derivative
    
    state0 = [1e-8, 0.0, 0.0, 0.0]
    t_span = (0, t_final)
    t_eval = np.linspace(0, t_final, num_points)
    
    def force_func(t):
        F1 = 3.5 * t
        F2 = 0.0
        return np.array([F1, F2])
    
    sol = solve_ivp(state_derivative, t_span, state0, t_eval=t_eval,
                    args=(force_func,), method='RK45')
    
    # محاسبه مقادیر نیروها در طول زمان برای ارسال به بخش رسم نمودار
    F1_vals = 3.5 * sol.t
    F2_vals = np.zeros_like(sol.t)
    F3_vals = np.zeros_like(sol.t)
    
    return sol.t, sol.y, F1_vals, F2_vals, F3_vals


# ─────────────────────────────────────────────────────────────────────────────
# 4. INVERSE DYNAMICS Example 1 (Fig. 12)
# ─────────────────────────────────────────────────────────────────────────────

def simulate_idr_example1():
    """
    Track circular trajectory: θ = π/12, φ = (π/5)·t
    Paper: Fig. 12
    """
    import numpy as np
    from dynamics import inverse_dynamics_3cables
    
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
    
    # آرایه برای ذخیره نیروی ۳ کابل
    F_array = np.zeros((3, num_points))
    
    for i in range(num_points):
        F = inverse_dynamics_3cables(theta_d[i], phi_d[i],
                                     dtheta_d[i], dphi_d[i],
                                     ddtheta_d[i], ddphi_d[i])
        F_array[:, i] = F
        
    return t, theta_d, phi_d, F_array

# ─────────────────────────────────────────────────────────────────────────────
# 5. INVERSE DYNAMICS Example 2 (Fig. 13)
# ─────────────────────────────────────────────────────────────────────────────

def simulate_idr_example2():
    """
    Track trajectory: θ = (π/4)·t, φ = π/6
    Paper: Fig. 13
    """
    import numpy as np
    from dynamics import inverse_dynamics_3cables
    
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
    
    # آرایه برای ذخیره نیروی 3 کابل
    F_array = np.zeros((3, num_points))
    
    for i in range(num_points):
        F = inverse_dynamics_3cables(theta_d[i], phi_d[i],
                                     dtheta_d[i], dphi_d[i],
                                     ddtheta_d[i], ddphi_d[i])
        F_array[:, i] = F
    
    return t, theta_d, phi_d, F_array


# ─────────────────────────────────────────────────────────────────────────────
# BONUS: PID CONTROL (Fig. 14)
# ─────────────────────────────────────────────────────────────────────────────

class SimplePIDController:
    """Simple 1-DOF PID for bending angle θ."""
    
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_error = 0.0
        self.prev_error = 0.0
    
    def update(self, error, dt):
        """Compute control output for given error and time step."""
        self.integral_error += error * dt
        derivative_error = (error - self.prev_error) / dt if dt > 0 else 0.0
        
        output = self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative_error
        self.prev_error = error
        
        return output


def simulate_pid_control(setpoint_deg=15.53, t_final=8.0, dt=0.02):
    """
    PID control to reach bending angle setpoint (Fig. 14).
    Paper parameters: Kp=2.8, Ki=0.004, Kd=0.38
    Error is calculated in DEGREES to match the paper's tuning gains.
    """
    import numpy as np
    from scipy.integrate import solve_ivp
    from dynamics import mass_matrix, coriolis_matrix, stiffness_matrix, damping_matrix
    from params import r_cab
    
    Kp, Ki, Kd = 2.8, 0.004, 0.38
    integral_error = 0.0
    prev_error = 0.0
    
    t_eval = np.arange(0, t_final, dt)
    state = np.array([1e-8, 0.0, 0.0, 0.0])  # [θ, φ, θ̇, φ̇]
    
    history = np.zeros((4, len(t_eval)))
    force_history = np.zeros((3, len(t_eval)))
    
    for i, current_t in enumerate(t_eval):
        history[:, i] = state
        theta = state[0]
        
        # محاسبه خطا بر حسب درجه (الزامی برای مچ شدن با Kp=2.8)
        error = setpoint_deg - (theta * 180.0 / np.pi)
        
        # آپدیت PID
        integral_error += error * dt
        derivative_error = (error - prev_error) / dt if dt > 0 else 0.0
        output = Kp * error + Ki * integral_error + Kd * derivative_error
        prev_error = error
        
        # تولید نیروی کششی کابل 1 (تنش منفی نداریم)
        F1 = max(0.0, output)
        F_req = np.array([F1, 0.0, 0.0]) 
        
        force_history[:, i] = F_req
        
        # تعریف دینامیک لوکال برای پشتیبانی از ماتریس 3 کابله در حلگر ODE
        def state_derivative_pid(t_inner, y):
            th, ph, th_dot, ph_dot = y
            if abs(th) < 1e-8: th = 1e-8
            
            M = mass_matrix(th)
            C = coriolis_matrix(th)
            K = stiffness_matrix()
            B = damping_matrix()
            
            q = np.array([th, ph])
            q_dot = np.array([th_dot, ph_dot])
            vel = np.array([th_dot**2, th_dot*ph_dot, ph_dot**2])
            
            # ماتریس D برای 3 کابل
            gam1, gam2, gam3 = 0.0, 2*np.pi/3, 4*np.pi/3
            D3 = np.array([
                [r_cab * np.cos(gam1 - ph), r_cab * np.cos(gam2 - ph), r_cab * np.cos(gam3 - ph)],
                [r_cab * th * np.sin(gam1 - ph), r_cab * th * np.sin(gam2 - ph), r_cab * th * np.sin(gam3 - ph)]
            ])
            
            rhs = D3 @ F_req - C @ vel - K @ q - B @ q_dot
            try:
                q_ddot = np.linalg.solve(M, rhs)
            except np.linalg.LinAlgError:
                q_ddot = np.linalg.lstsq(M, rhs, rcond=None)[0]
                
            return [th_dot, ph_dot, q_ddot[0], q_ddot[1]]
        
        # شبیه‌سازی دقیق گسسته (Digital Control Loop Hold)
        if i < len(t_eval) - 1:
            sol = solve_ivp(state_derivative_pid, [current_t, current_t + dt], state, method='RK45')
            state = sol.y[:, -1]
            
    return t_eval, history, force_history
