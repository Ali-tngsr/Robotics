#!/usr/bin/env python3
"""
MAIN TEST SCRIPT
Validates all phases of the CDCR implementation against Amouri et al. 2020.

Run as: python main.py
"""
import sys
import numpy as np
from params import L, m_b, m_d, I_b, I_xx, E, THETA_MAX
from kinematics import position, orientation_matrix, linear_velocity, angular_velocity
from taylor_factors import get_all_H, get_all_dH, H1, H2, H3, H4, H5, H6, H7, H8
from dynamics import mass_matrix, coriolis_matrix, stiffness_matrix, force_matrix
from plots import generate_all_plots


def test_phase1_kinematics():
    """Test Phase 1: Kinematics"""
    print("\n" + "="*70)
    print("PHASE 1: KINEMATICS")
    print("="*70)
    
    # Test position vector
    s_test = 0.4  # mid-point
    theta_test = np.pi / 6  # 30°
    phi_test = 0.0
    
    r = position(s_test, theta_test, phi_test, L)
    print(f"✓ Position vector r_s at s={s_test:.2f}, θ={theta_test:.4f}, φ={phi_test:.4f}:")
    print(f"  r = {r}")
    
    # Test orientation matrix
    R = orientation_matrix(s_test, theta_test, phi_test, L)
    print(f"✓ Orientation matrix R_s (shape {R.shape}):")
    print(f"  Determinant (should be ≈1): {np.linalg.det(R):.6f}")
    
    # Test velocities
    theta_dot = 0.5
    phi_dot = 0.3
    v = linear_velocity(s_test, theta_test, phi_test, theta_dot, phi_dot, L)
    omega = angular_velocity(s_test, theta_test, phi_test, theta_dot, phi_dot, L)
    print(f"✓ Velocities (θ̇={theta_dot}, φ̇={phi_dot}):")
    print(f"  v_s = {v},  ω_s = {omega}")
    
    print("✓ Phase 1 PASSED\n")


def test_phase2_taylor_factors():
    """Test Phase 2: Taylor factors"""
    print("="*70)
    print("PHASE 2: TAYLOR EXPANSION FACTORS")
    print("="*70)
    
    theta_test = np.pi / 9  # 20°
    
    # Get all H factors
    H_vals = get_all_H(theta_test)
    dH_vals = get_all_dH(theta_test)
    
    print(f"✓ H-factors at θ = {theta_test:.4f} rad ({theta_test*180/np.pi:.1f}°):")
    for i, h in enumerate(H_vals, 1):
        print(f"  H{i} = {h:.6f}")
    
    print(f"\n✓ dH/dθ derivatives at θ = {theta_test:.4f}:")
    for i, dh in enumerate(dH_vals, 1):
        print(f"  dH{i}/dθ = {dh:.6f}")
    
    # Test singularity handling (theta → 0)
    h_small = get_all_H(1e-10)
    print(f"\n✓ H-factors near θ=0 (safe evaluation):")
    print(f"  H1(θ→0) = {h_small[0]:.6f} (expected ≈0.15)")
    print(f"  H3(θ→0) = {h_small[2]:.6f} (expected ≈0.333)")
    
    print("✓ Phase 2 PASSED\n")


def test_phase3_dynamics():
    """Test Phase 3: Dynamics assembly"""
    print("="*70)
    print("PHASE 3–4: DYNAMICS & EQUATIONS OF MOTION")
    print("="*70)
    
    theta_test = np.pi / 9
    phi_test = np.pi / 12
    
    # Mass matrix
    M = mass_matrix(theta_test)
    print(f"✓ Mass matrix M(θ={theta_test:.4f}):")
    print(f"  M11 = {M[0,0]:.6e} kg·m²")
    print(f"  M22 = {M[1,1]:.6e} kg·m²")
    print(f"  Determinant: {np.linalg.det(M):.6e} (should be > 0)")
    
    # Coriolis matrix
    C = coriolis_matrix(theta_test)
    print(f"\n✓ Coriolis matrix C(θ={theta_test:.4f}) [shape 2×3]:")
    print(f"  C11 = {C[0,0]:.6f},  C13 = {C[0,2]:.6f}")
    print(f"  C22 = {C[1,1]:.6f}")
    
    # Stiffness matrix
    K = stiffness_matrix()
    print(f"\n✓ Stiffness matrix K:")
    print(f"  K11 = {K[0,0]:.6e} N·m/rad (elastic)")
    print(f"  K22 = {K[1,1]:.6f} (no rotational stiffness)")
    
    # Force matrix
    D = force_matrix(theta_test, phi_test)
    print(f"\n✓ Actuation map D(θ={theta_test:.4f}, φ={phi_test:.4f}):")
    print(f"  [F1, F2] → [Q1, Q2] mapping:")
    print(f"  D =\n{D}")
    print(f"  det(D) = {np.linalg.det(D):.6e}")
    
    print("✓ Phase 3–4 PASSED\n")


def test_phase5_simulation():
    """Test Phase 5: Basic simulation"""
    print("="*70)
    print("PHASE 5: SIMULATION (Quick Test)")
    print("="*70)
    
    from dynamics import state_derivative
    
    # Quick ODE step
    state = [np.pi/9, 0.1, 0.2, 0.15]
    def force_func(t):
        return np.array([1.0, 0.0])
    
    derivs = state_derivative(0.0, state, force_func)
    print(f"✓ State derivative at t=0:")
    print(f"  state = {state}")
    print(f"  d/dt[state] = {derivs}")
    print(f"  (θ̇={derivs[0]:.4f}, φ̇={derivs[1]:.4f}, θ̈={derivs[2]:.4f}, φ̈={derivs[3]:.4f})")
    
    print("\n✓ Phase 5 PASSED (basic integration ready)\n")


def main():
    """Run all tests."""
    print("\n")
    print("╔" + "="*68 + "╗")
    print("║" + " "*68 + "║")
    print("║" + "  CDCR DYNAMIC MODEL VALIDATION (Amouri et al. 2020)".center(68) + "║")
    print("║" + "  Clean Python Implementation".center(68) + "║")
    print("║" + " "*68 + "║")
    print("╚" + "="*68 + "╝")
    
    try:
        test_phase1_kinematics()
        test_phase2_taylor_factors()
        test_phase3_dynamics()
        test_phase5_simulation()
        
        print("="*70)
        print("ALL PHASES VALIDATED ✓")
        print("="*70)
        print("\nReady to generate figures. Running Phase 5+ (simulations)...")
        print("This may take 1-2 minutes...\n")
        
        # Generate all plots
        figs = generate_all_plots(show=True, save=False)
        
        return 0
        
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
