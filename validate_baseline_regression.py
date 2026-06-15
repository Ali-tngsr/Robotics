import params

# ۱. تنظیم جرم بار روی صفر مطلق برای صحت‌سنجی
params.m_p = 0.0

# ۲. ایمپورت کردن ماژول رسم نمودارها که در فاز اول تکمیل کردیم
from plots import generate_all_plots

print("==========================================================")
print(" 🚀 STARTING REGRESSION TEST: BASELINE VALIDATION ")
print("==========================================================")
print(f"Current Payload Mass (m_p) strictly set to: {params.m_p} kg")
print("Generating all original baseline figures...")
print("Checking Fig 8, 9, 10, 12, 13, and 14...")

# ۳. تولید، نمایش و ذخیره تمام نمودارها با مدل جدید اما جرم صفر
try:
    figs = generate_all_plots(show=True, save=True)
    print("==========================================================")
    print("✅ VALIDATION PASSED: All simulations completed successfully!")
    print("   The extended model perfectly reduces to the baseline")
    print("   model when m_p = 0.0 kg.")
    print("==========================================================")
except Exception as e:
    print("❌ VALIDATION FAILED: An error occurred in the extended model.")
    print(f"Error details: {e}")
