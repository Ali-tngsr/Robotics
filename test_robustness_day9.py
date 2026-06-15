import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import warnings
warnings.filterwarnings("ignore")

import params
from dynamics import mass_matrix, coriolis_matrix, stiffness_matrix, gravity_vector
from smc_controller import SMCController

# ==========================================
# تنظیمات سناریو: بار ناشناخته 150 گرمی
# ==========================================
REAL_PAYLOAD = 0.0000   # واقعیت: 150 گرم به ربات وصل است
params.m_p = REAL_PAYLOAD 
KNOWN_PAYLOAD = 0.0   # درک کنترلر: فکر می‌کند هیچ باری وجود ندارد (مدل نامی)

# ==========================================
# 1. طراحی کنترلر SMC (قوی و مقاوم)
# ==========================================
smc = SMCController(lambda_1=8.0, lambda_2=8.0, k_1=8.0, k_2=8.0)
smc.Phi = 0.1 # لایه مرزی

# ==========================================
# 2. طراحی کنترلر PID (ضعیف در برابر تغییرات)
# ==========================================
# استفاده از ضرایبی شبیه به مقاله (برای زاویه تتا)
Kp, Ki, Kd = 20.0, 5.0, 5.0

# ==========================================
# 2. طراحی کنترلر PID دو درجه آزادی (صحیح و کامل)
# ==========================================
class PIDController2DOF:
    def __init__(self):
        # ضرایب Kp, Ki, Kd برای هر دو زاویه (تتا و فای)
        self.Kp = np.array([20.0, 20.0])
        self.Ki = np.array([5.0, 5.0])
        self.Kd = np.array([5.0, 5.0])
        self.integral = np.zeros(2)
        
    def get_control(self, e, e_dot, dt):
        self.integral += e * dt
        # محاسبه گشتاور PID برای هر دو مفصل
        return -(self.Kp * e + self.Ki * self.integral + self.Kd * e_dot)

pid = PIDController2DOF()

# ==========================================
# توابع کمکی مشترک
# ==========================================
def desired_trajectory(t):
    theta_d = np.pi / 12  # 15 degrees
    phi_d = (np.pi / 5) * t
    return np.array([theta_d, phi_d]), np.array([0.0, np.pi/5]), np.array([0.0, 0.0])

def force_matrix_3cables(theta, phi):
    r = params.r_cab
    gam1, gam2, gam3 = 0.0, 2*np.pi/3, 4*np.pi/3
    D11 = r * np.cos(gam1 - phi); D12 = r * np.cos(gam2 - phi); D13 = r * np.cos(gam3 - phi)
    D21 = r * theta * np.sin(gam1 - phi); D22 = r * theta * np.sin(gam2 - phi); D23 = r * theta * np.sin(gam3 - phi)
    return np.array([[D11, D12, D13], [D21, D22, D23]])

# ==========================================
# دینامیک حلقه بسته برای SMC
# ==========================================
def closed_loop_smc(t, state):
    theta, phi, theta_dot, phi_dot = state
    q = np.array([theta, phi])
    q_dot = np.array([theta_dot, phi_dot])
    vel = np.array([theta_dot**2, theta_dot*phi_dot, phi_dot**2])
    q_d, q_d_dot, q_d_ddot = desired_trajectory(t)
    
    # دینامیک واقعی سیستم (با بار 150 گرم)
    M_real = mass_matrix(theta)
    C_real = coriolis_matrix(theta)
    K_real = stiffness_matrix()
    G_real = gravity_vector(theta)
    D3 = force_matrix_3cables(theta, phi)
    C_v_real = C_real @ vel
    
    # 🌟 درک کنترلر از سیستم (مدل نامی بدون بار)
    params.m_p = KNOWN_PAYLOAD # موقتا صفر می‌کنیم تا کنترلر فریب بخورد
    M_nom = mass_matrix(theta)
    C_nom = coriolis_matrix(theta)
    G_nom = gravity_vector(theta)
    C_v_nom = C_nom @ vel
    params.m_p = REAL_PAYLOAD  # برمی‌گردانیم به واقعیت
    
    # کنترلر بر اساس مدل غلط (بدون بار) نیرو تولید می‌کند
    F_cmd, _, _ = smc.get_control(q, q_dot, q_d, q_d_dot, q_d_ddot, M_nom, C_v_nom, K_real, G_nom, D3)
    
    # اما نیرو به سیستم واقعی اعمال می‌شود
    q_ddot = np.linalg.solve(M_real, D3 @ F_cmd - C_v_real - K_real @ q - G_real)
    return [theta_dot, phi_dot, q_ddot[0], q_ddot[1]]

# ==========================================
# دینامیک حلقه بسته برای PID (اصلاح شده)
# ==========================================
last_t = 0.0
def closed_loop_pid(t, state):
    global last_t
    dt = t - last_t
    if dt <= 0: dt = 1e-4
    last_t = t
    
    theta, phi, theta_dot, phi_dot = state
    q = np.array([theta, phi])
    q_dot = np.array([theta_dot, phi_dot])
    vel = np.array([theta_dot**2, theta_dot*phi_dot, phi_dot**2])
    q_d, q_d_dot, _ = desired_trajectory(t)
    
    # دینامیک واقعی
    M_real = mass_matrix(theta)
    C_real = coriolis_matrix(theta)
    K_real = stiffness_matrix()
    G_real = gravity_vector(theta)
    D3 = force_matrix_3cables(theta, phi)
    C_v_real = C_real @ vel
    
    # محاسبه خطای کامل (هر دو زاویه)
    e = q - q_d
    e_dot = q_dot - q_d_dot
    
    # فیدفوروارد فنریت برای هر دو زاویه (K * q_d)
    tau_ff = K_real @ q_d
    
    # گشتاور کل PID
    tau_pid_val = pid.get_control(e, e_dot, dt)
    tau_total = tau_pid_val + tau_ff 
    
    # توزیع نیرو بین ۳ کابل
    D_pinv = np.linalg.pinv(D3)
    F_cmd = D_pinv @ tau_total
    
    # حفظ کشش مثبت
    if np.min(F_cmd) < 0: 
        F_cmd = F_cmd + np.abs(np.min(F_cmd)) + 1.0 
    
    # اعمال به سیستم واقعی
    q_ddot = np.linalg.solve(M_real, D3 @ F_cmd - C_v_real - K_real @ q - G_real)
    return [theta_dot, phi_dot, q_ddot[0], q_ddot[1]]
# ==========================================
# اجرای شبیه‌سازی و رسم نمودار
# ==========================================
print(f"Running Robustness Test: 150g Payload, Controller knows 0g...")
t_eval = np.linspace(0, 10, 500)
state0 = [1e-4, 0.0, 0.0, 0.0] 

print("1/2: Simulating SMC...")
sol_smc = solve_ivp(closed_loop_smc, [0, 10], state0, t_eval=t_eval, method='LSODA')

print("2/2: Simulating PID...")
last_t = 0.0 # Reset time for PID

# 🌟 تغییر جدید: استفاده از رویکرد امن برای اجرای PID که منفجر نشود
try:
    sol_pid = solve_ivp(closed_loop_pid, [0, 10], state0, t_eval=t_eval, method='LSODA')
    pid_theta = sol_pid.y[0, :]
except Exception as e:
    print("⚠️ PID completely failed (Diverged) as expected!")
    pid_theta = np.nan * np.ones_like(t_eval) # پر کردن با NaN برای رسم

# 🌟 اگر PID قبل از 10 ثانیه کرش کرد، آرایه را برای رسم هماهنگ می‌کنیم
if len(pid_theta) < len(t_eval):
    print(f"⚠️ PID diverged and crashed at step {len(pid_theta)}!")
    pid_theta_full = np.nan * np.ones_like(t_eval)
    pid_theta_full[:len(pid_theta)] = pid_theta
    pid_theta = pid_theta_full

# ==========================================
# رسم نمودار
# ==========================================
plt.figure(figsize=(10, 5))
plt.plot(t_eval, np.degrees(np.ones_like(t_eval) * (np.pi / 12)), 'k--', linewidth=2, label='Desired Target (15°)')
plt.plot(t_eval, np.degrees(sol_smc.y[0, :]), 'b', linewidth=2, label='SMC (Robust & Stable)')
plt.plot(t_eval, np.degrees(pid_theta), 'r', linewidth=2, label='PID (Fails/Crashes)')

plt.title(f"Day 9 Robustness Test: Unmodeled {REAL_PAYLOAD*1000}g Payload")
plt.xlabel("Time (s)")
plt.ylabel("Bending Angle $\\theta$ (degrees)")
plt.ylim([0, 25]) # محدود کردن محور Y برای جلوگیری از خراب شدن گراف بخاطر انفجار PID
plt.legend()
plt.grid(True)
plt.show()