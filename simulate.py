import numpy as np
from smc_controller import SlidingModeController
from pid_controller import PIDController
from dynamics import state_derivative_payload
from scipy.integrate import solve_ivp

def run_smc_tracking_with_payload():
    m_p_true = 0.050    
    m_p_hat  = 0.030    
    
    Lambda = np.array([[20.0, 0.0], [0.0, 20.0]])
    K_sw   = np.array([[1.5, 0.0], [0.0, 1.5]])
    phi_bound = np.array([0.5, 0.5]) 
                       
    smc = SlidingModeController(Lambda, K_sw, m_p_hat, phi_bound)
    
    def closed_loop_ode(t, state):
        theta_d = np.pi / 12
        phi_d = (np.pi / 5) * t
        desired_state = [theta_d, phi_d, 0.0, np.pi / 5, 0.0, 0.0]
        
        F, _, _ = smc.compute_control_law(state, desired_state)
        force_func = lambda t_val: F
        return state_derivative_payload(t, state, force_func, m_p_true)

    t_span = (0.0, 3.0)
    t_eval = np.arange(0, 3.0, 0.01)
    state0 = [np.pi/12, 0.0, 0.0, np.pi/5]
    
    sol = solve_ivp(closed_loop_ode, t_span, state0, t_eval=t_eval, method='BDF', rtol=1e-3, atol=1e-3)
    t = sol.t
    history = sol.y
    
    desired_history = np.zeros((2, len(t)))
    force_history = np.zeros((2, len(t)))
    error_history = np.zeros((2, len(t)))
    
    for i, current_t in enumerate(t):
        current_state = history[:, i]
        theta_d = np.pi / 12
        phi_d = (np.pi / 5) * current_t
        desired_state = [theta_d, phi_d, 0.0, np.pi / 5, 0.0, 0.0]
        desired_history[:, i] = [theta_d, phi_d]
        
        F, S, e = smc.compute_control_law(current_state, desired_state)
        force_history[:, i] = F
        error_history[:, i] = e

    return t, history, desired_history, force_history, error_history

def run_pid_tracking_with_payload():
    m_p_true = 0.050
    Kp = np.array([[30.0, 0.0], [0.0, 30.0]])
    Ki = np.array([[5.0, 0.0], [0.0, 5.0]])
    Kd = np.array([[10.0, 0.0], [0.0, 10.0]])
    pid = PIDController(Kp, Ki, Kd)
    
    dt = 0.01
    t = np.arange(0, 3.0, dt)
    state = np.array([np.pi/12, 0.0, 0.0, np.pi/5])
    
    history = np.zeros((4, len(t)))
    desired_history = np.zeros((2, len(t)))
    error_history = np.zeros((2, len(t)))
    
    for i, current_t in enumerate(t):
        state[0] = np.clip(state[0], 1e-6, np.pi/2)
        state[2] = np.clip(state[2], -15.0, 15.0) 
        state[3] = np.clip(state[3], -15.0, 15.0) 
        
        history[:, i] = state
        
        theta_d = np.pi / 12
        phi_d = (np.pi / 5) * current_t
        desired_state = [theta_d, phi_d, 0.0, np.pi / 5, 0.0, 0.0]
        desired_history[:, i] = [theta_d, phi_d]
        
        F, e = pid.compute_control_law(state, desired_state, dt)
        error_history[:, i] = e
        
        force_func = lambda t_val: F
        sol = solve_ivp(state_derivative_payload, [current_t, current_t + dt], state, 
                        args=(force_func, m_p_true), method='RK45')
        state = sol.y[:, -1]
        
    return t, history, desired_history, error_history
# ---------------------------------------------------------
# 1. Static Equilibrium Analysis (Fig. 8)
# ---------------------------------------------------------
def run_static_equilibrium():
    state0 = [np.pi/4, 0.0, 0.0, 0.0]
    t_span = (0, 40)
    t_eval = np.linspace(t_span[0], t_span[1], 1000)
    force_func = lambda t: np.array([0.0, 0.0])
    
    # Passing m_p = 0.0 to the universal physics engine
    sol = solve_ivp(state_derivative_payload, t_span, state0, t_eval=t_eval, args=(force_func, 0.0), method='RK45')
    return sol.t, sol.y

def run_fdr_example_1():
    state0 = [1e-6, 0.0, 0.0, 0.0]
    t_span = (0, 40)
    t_eval = np.linspace(t_span[0], t_span[1], 1000)
    force_func = lambda t: np.array([5.0, 0.0])
    
    # Passing m_p = 0.0 to the universal physics engine
    sol = solve_ivp(state_derivative_payload, t_span, state0, t_eval=t_eval, args=(force_func, 0.0), method='RK45')
    return sol.t, sol.y

# ---------------------------------------------------------
# 3. Inverse Dynamic Response - Example 1 (Fig. 12)
# ---------------------------------------------------------
def run_idr_example_1():
    """
    Tracks a spatial circular trajectory[cite: 598].
    theta = pi/12, phi = (pi/5)*t
    """
    t = np.linspace(0, 10, 500)
    theta = np.ones_like(t) * (np.pi / 12)
    phi = (np.pi / 5) * t
    
    dtheta = np.zeros_like(t)
    ddtheta = np.zeros_like(t)
    
    dphi = np.ones_like(t) * (np.pi / 5)
    ddphi = np.zeros_like(t)
    
    Q_req = np.zeros((2, len(t)))
    
    for i in range(len(t)):
        M, C, K, D = assemble_matrices(theta[i], phi[i])
        
        q_ddot = np.array([ddtheta[i], ddphi[i]])
        v_vec = np.array([dtheta[i]**2, dphi[i]**2, dtheta[i]*dphi[i]])
        q = np.array([theta[i], phi[i]])
        
        # M*q_ddot + C*v + K*q = Q
        Q_req[:, i] = M @ q_ddot + C @ v_vec + K @ q

    return t, theta, phi, Q_req

# ---------------------------------------------------------
# 4. PID Controller (Fig. 14)
# ---------------------------------------------------------
def run_pid_control():
    """
    Implements a PID controller to reduce oscillations from FDR Example 1.
    Setpoint is theta = 15.53 degrees. Uses the universal physics engine with 0g payload.
    """
    # PID Parameters from the paper
    Kp, Ki, Kd = 2.8, 0.004, 0.38
    
    setpoint = 15.53 * (np.pi / 180)  # Convert to radians
    
    dt = 0.02
    t = np.arange(0, 5.0, dt)
    state = np.array([1e-6, 0.0, 0.0, 0.0])
    
    history = np.zeros((4, len(t)))
    force_history = np.zeros(len(t))
    
    integral_err = 0.0
    prev_err = setpoint - state[0]
    
    for i, current_t in enumerate(t):
        history[:, i] = state
        
        # Calculate Errors
        err = setpoint - state[0]
        integral_err += err * dt
        derivative_err = (err - prev_err) / dt
        
        # PID Output (Force on cable 1)
        F1 = max(0, Kp * err + Ki * integral_err + Kd * derivative_err)
        force_history[i] = F1
        
        force_func = lambda t_val: np.array([F1, 0.0])
        
        # THE FIX: Route through state_derivative_payload with m_p = 0.0
        sol = solve_ivp(state_derivative_payload, [current_t, current_t + dt], state, 
                        args=(force_func, 0.0), method='RK45')
        state = sol.y[:, -1]
        prev_err = err
        
    return t, history, force_history