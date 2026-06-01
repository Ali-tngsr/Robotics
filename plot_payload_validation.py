import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# ایمپورت ماژول‌های پروژه
import params
from dynamics import state_derivative

def generate_payload_validation_plot():
    print("Simulating Payload Forward Dynamics for Validation Plot...")
    
    # تنظیمات شبیه‌سازی
    t_span = (0.0, 15.0)
    t_eval = np.linspace(t_span[0], t_span[1], 1000)
    y0 = [1e-3, 0.0, 0.0, 0.0]  # شرایط اولیه
    
    # اعمال یک نیروی ثابت 5 نیوتنی به کابل اول برای خم کردن ربات
    def constant_force(t):
        return np.array([5.0, 0.0])
    
    # حالت‌های مختلف جرم بار (Baseline تا 150 گرم)
    mass_cases = [0.0, 0.05, 0.10, 0.15]
    colors = ['#2ca02c', '#1f77b4', '#ff7f0e', '#d62728'] # سبز، آبی، نارنجی، قرمز
    
    plt.figure(figsize=(10, 6))
    
    for m, color in zip(mass_cases, colors):
        params.m_p = m  # تغییر زنده جرم بار در پارامترها
        
        def odefunc(t, y):
            return state_derivative(t, y, constant_force)
            
        sol = solve_ivp(odefunc, t_span, y0, t_eval=t_eval, method='RK45')
        
        theta_deg = np.rad2deg(sol.y[0, :])
        stable_angle = theta_deg[-1]
        
        label_str = f"Baseline ($m_p$ = 0 kg)" if m == 0.0 else f"Payload = {m*1000:.0f} g"
        plt.plot(sol.t, theta_deg, color=color, linewidth=2, 
                 label=f"{label_str} (Settles at {stable_angle:.1f}°)")
        print(f"Computed {label_str} -> Steady-State Angle: {stable_angle:.2f}°")

    # زیباسازی و تنظیمات نمودار
    plt.title("Forward Dynamics Validation: Effect of Payload Mass under Constant 5N Tension", fontsize=14, pad=15)
    plt.xlabel("Time (s)", fontsize=12)
    plt.ylabel("Bending Angle $\\theta$ (degrees)", fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(loc='lower right', fontsize=11)
    
    # ذخیره نمودار در پوشه (در صورت وجود پوشه figures)
    import os
    if not os.path.exists('figures'):
        os.makedirs('figures')
    save_path = 'figures/fig_extension_payload_validation.png'
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    plt.tight_layout()
    plt.show()
    print(f"\nPlot successfully saved to: {save_path}")

if __name__ == '__main__':
    generate_payload_validation_plot()
