import numpy as np

class SMCController:
    # اضافه شدن K_1 و K_2 برای قدرت هل دادن ربات به سمت مسیر
    def __init__(self, lambda_1=8.0, lambda_2=8.0, k_1=5.0, k_2=5.0):
        self.Lambda = np.diag([lambda_1, lambda_2])
        self.K_smc = np.diag([k_1, k_2])
        self.Phi = 0.2  # ضخامت لایه مرزی برای حذف لرزش (Chattering)

    def get_control(self, q, q_dot, q_d, q_d_dot, q_d_ddot, M, C_v, K, G, D):
        e = q - q_d
        e_dot = q_dot - q_d_dot
        s = e_dot + self.Lambda @ e
        
        # ۱. کنترل معادل (F_eq) - لغو دینامیک
        q_r_ddot = q_d_ddot - self.Lambda @ e_dot
        tau_eq = M @ q_r_ddot + C_v + K @ q + G
        
        # ۲. قانون سوئیچینگ (F_sw) - هل دادن ربات به سمت مسیر
        # استفاده از تابع clip به جای sign برای شبیه‌سازی sat(s/Phi)
        sat_s = np.clip(s / self.Phi, -1.0, 1.0)
        tau_sw = -self.K_smc @ sat_s
        
        # ۳. گشتاور کل
        tau_total = tau_eq + tau_sw
        
        # ۴. تبدیل گشتاور به نیروی کابل‌ها
        D_pinv = np.linalg.pinv(D)
        F_cmd = D_pinv @ tau_total

        # ۵. حفظ کشش مثبت (Null Space)
        if np.min(F_cmd) < 0:
            F_cmd = F_cmd + np.abs(np.min(F_cmd)) + 1.0 

        return F_cmd, s, e