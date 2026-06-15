"""
Smooth Sliding Mode Controller (SMC)
Uses hyperbolic tangent (tanh) to completely eliminate chattering.
"""
import numpy as np

class SMCController:
    # پارامترها تیون شدند: بهره کمتر، لایه مرزی نرم‌تر
    def __init__(self, lambda_c=5.0, K_s=4.0, epsilon=0.2):
        self.lambda_c = lambda_c
        self.K_s = K_s
        self.epsilon = epsilon

    def update(self, error, error_dot):
        """Compute control output using Smooth Sliding Mode logic."""
        s = error_dot + self.lambda_c * error
        
        # استفاده از تابع تانژانت هیپربولیک (tanh) به جای sign یا sat خطی
        # این فرمول چترینگ را در شبیه‌سازی‌های گسسته-زمان کاملاً محو می‌کند
        smooth_sat = np.tanh(s / self.epsilon)
            
        output = self.K_s * smooth_sat
        return output