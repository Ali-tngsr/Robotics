"""
PHASE 2: TAYLOR EXPANSION FACTORS
H1–H8 factors appear in kinetic energy expressions.
Exact forms + Taylor approximations (4th order in θ).

The paper uses Taylor expansions to avoid singularities and reduce complexity.
All expressions valid for θ ∈ [-3π/5, 3π/5].
"""
import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# SAFE SWITCHING BETWEEN EXACT AND TAYLOR
# ─────────────────────────────────────────────────────────────────────────────

def _safe_eval(exact_func, taylor_func, theta, threshold=1e-3):
    """
    Evaluate function, switching to Taylor near singularity.
    
    Parameters
    ----------
    exact_func : callable  exact expression f(θ)
    taylor_func : callable  Taylor expansion f_taylor(θ)
    theta : float or array
    threshold : float  switch threshold |θ| < threshold
    
    Returns
    -------
    result : float or array
    """
    theta = np.asarray(theta, dtype=float)
    mask_taylor = np.abs(theta) < threshold
    
    result = np.where(mask_taylor,
                      taylor_func(theta),
                      exact_func(theta))
    return result


# ─────────────────────────────────────────────────────────────────────────────
# H1 — BACKBONE TRANSLATIONAL KINETIC ENERGY (θ̇² coefficient)
# Eqs. (10) exact, (12) Taylor
# ─────────────────────────────────────────────────────────────────────────────

def _H1_exact(theta):
    """Eq. (10): (θ³ + 6θ - 12sinθ + 6θcosθ) / θ⁵"""
    t = np.asarray(theta, dtype=float)
    num = t**3 + 6*t - 12*np.sin(t) + 6*t*np.cos(t)
    return num / t**5


def _H1_taylor(theta):
    """Eq. (12): θ⁴/8640 - θ²/168 + 3/20"""
    t = np.asarray(theta, dtype=float)
    return t**4/8640 - t**2/168 + 3/20


def H1(theta):
    return _safe_eval(_H1_exact, _H1_taylor, theta)


# ─────────────────────────────────────────────────────────────────────────────
# H2 — BACKBONE TRANSLATIONAL KINETIC ENERGY (φ̇² coefficient)
# Eqs. (11) exact, (13) Taylor
# ─────────────────────────────────────────────────────────────────────────────

def _H2_exact(theta):
    """Eq. (11): (6θ - 8sinθ + sin(2θ)) / θ³"""
    t = np.asarray(theta, dtype=float)
    num = 6*t - 8*np.sin(t) + np.sin(2*t)
    return num / t**3


def _H2_taylor(theta):
    """Eq. (13): -θ⁴/42 + θ²/5"""
    t = np.asarray(theta, dtype=float)
    return -t**4/42 + t**2/5


def H2(theta):
    return _safe_eval(_H2_exact, _H2_taylor, theta)


# ─────────────────────────────────────────────────────────────────────────────
# H3 — BACKBONE ROTATIONAL KINETIC ENERGY (θ̇² coefficient)
# Eq. (25) exact
# ─────────────────────────────────────────────────────────────────────────────

def _H3_exact(theta):
    """Eq. (25): 5/8 - 15/(64θ²) + cos²θ/(2θ²) - sin(2θ)/(8θ³) - sin(4θ)/(256θ³)"""
    t = np.asarray(theta, dtype=float)
    return (5/8 
            - 15/(64*t**2) 
            + np.cos(t)**2/(2*t**2)
            - np.sin(2*t)/(8*t**3)
            - np.sin(4*t)/(256*t**3))


def _H3_taylor(theta):
    """Taylor expansion of H3 around θ=0"""
    t = np.asarray(theta, dtype=float)
    return 1/3 + t**2/60 - t**4/840


def H3(theta):
    return _safe_eval(_H3_exact, _H3_taylor, theta)


# ─────────────────────────────────────────────────────────────────────────────
# H4 — BACKBONE ROTATIONAL KINETIC ENERGY (φ̇² coefficient)
# Eq. (26) exact
# ─────────────────────────────────────────────────────────────────────────────

def _H4_exact(theta):
    """Eq. (26): 1/2 - θ·sin(2θ)/(4θ²)"""
    t = np.asarray(theta, dtype=float)
    return 1/2 - t*np.sin(2*t)/(4*t**2)


def _H4_taylor(theta):
    """Taylor expansion of H4 around θ=0"""
    t = np.asarray(theta, dtype=float)
    return t**2/6 - t**4/60


def H4(theta):
    return _safe_eval(_H4_exact, _H4_taylor, theta)


# ─────────────────────────────────────────────────────────────────────────────
# H5 — DISK TRANSLATIONAL KINETIC ENERGY (θ̇² coefficient)
# Eq. (27) exact — sum over k=1..10 disks at s_k = k·L/10
# ─────────────────────────────────────────────────────────────────────────────

def _H5_exact(theta, N=10):
    """
    Eq. (27): Complex form summing over disks.
    Numerical integration approach: sum ∫ (dr_k/dθ)² over all disk positions.
    """
    t = np.asarray(theta, dtype=float)
    scalar = t.ndim == 0
    t = np.atleast_1d(t)
    result = np.zeros_like(t)
    
    for i, th in enumerate(t):
        if np.abs(th) < 1e-8:
            # Small angle: (1-cosθ_k)²/θ² + sin²θ_k/θ² ≈ (θ_k/2)² + θ_k² = 5θ_k²/4
            s_sum = sum((k/N)**2 * (5/4) for k in range(1, N+1))
            result[i] = s_sum / (N**2)
        else:
            s_sum = 0.0
            for k in range(1, N+1):
                frac = k / N  # s_k / L
                th_k = frac * th
                term = (1 - np.cos(th_k))**2 / th**2 + np.sin(th_k)**2 / th**2
                s_sum += term
            result[i] = s_sum / (N**2)
    
    return result[0] if scalar else result


def _H5_taylor(theta, N=10):
    """Taylor approximation of H5"""
    t = np.asarray(theta, dtype=float)
    const = sum((k/N)**2 for k in range(1, N+1)) / (N**2)  # ≈ 0.3717
    return np.full_like(t, const, dtype=float) + t**2/12 * const


def H5(theta, N=10):
    return _safe_eval(lambda x: _H5_exact(x, N),
                      lambda x: _H5_taylor(x, N),
                      theta)


# ─────────────────────────────────────────────────────────────────────────────
# H6 — DISK TRANSLATIONAL KINETIC ENERGY (φ̇² coefficient)
# Eq. (28) exact
# ─────────────────────────────────────────────────────────────────────────────

def _H6_exact(theta, N=10):
    """Eq. (28): Sum over disks of (1-cosθ_k)²/θ²"""
    t = np.asarray(theta, dtype=float)
    scalar = t.ndim == 0
    t = np.atleast_1d(t)
    result = np.zeros_like(t)
    
    for i, th in enumerate(t):
        if np.abs(th) < 1e-8:
            s_sum = sum((k/N)**2 * (k/N)**2 / 4 for k in range(1, N+1))
            result[i] = s_sum / (N**2)
        else:
            s_sum = 0.0
            for k in range(1, N+1):
                frac = k / N
                th_k = frac * th
                term = (1 - np.cos(th_k))**2 / th**2
                s_sum += term
            result[i] = s_sum / (N**2)
    
    return result[0] if scalar else result


def _H6_taylor(theta, N=10):
    t = np.asarray(theta, dtype=float)
    const = sum((k/N)**4 for k in range(1, N+1)) / (4 * N**2)
    return np.full_like(t, const, dtype=float)


def H6(theta, N=10):
    return _safe_eval(lambda x: _H6_exact(x, N),
                      lambda x: _H6_taylor(x, N),
                      theta)


# ─────────────────────────────────────────────────────────────────────────────
# H7 — DISK ROTATIONAL KINETIC ENERGY (θ̇² coefficient)
# Eq. (29) exact
# ─────────────────────────────────────────────────────────────────────────────

def _H7_exact(theta, N=10):
    """Eq. (29): Sum over disks of angular velocity squared contribution"""
    t = np.asarray(theta, dtype=float)
    scalar = t.ndim == 0
    t = np.atleast_1d(t)
    result = np.zeros_like(t)
    
    for i, th in enumerate(t):
        s_sum = 0.0
        for k in range(1, N+1):
            frac = k / N
            th_k = frac * th
            # ω_k² from θ̇² part: sin²(θ_k)/θ² + cos²(θ_k)·(s_k/L)²
            term = (np.sin(th_k) / th)**2 + np.cos(th_k)**2 * frac**2
            s_sum += term
        result[i] = s_sum / (N**2)
    
    return result[0] if scalar else result


def _H7_taylor(theta, N=10):
    t = np.asarray(theta, dtype=float)
    const = sum(frac**2 for frac in [k/N for k in range(1, N+1)]) / (N**2)
    return np.full_like(t, const, dtype=float)


def H7(theta, N=10):
    return _safe_eval(lambda x: _H7_exact(x, N),
                      lambda x: _H7_taylor(x, N),
                      theta)


# ─────────────────────────────────────────────────────────────────────────────
# H8 — DISK ROTATIONAL KINETIC ENERGY (φ̇² coefficient)
# Eq. (30) exact
# ─────────────────────────────────────────────────────────────────────────────

def _H8_exact(theta, N=10):
    """Eq. (30): Sum over disks of sin²(θ_k)"""
    t = np.asarray(theta, dtype=float)
    scalar = t.ndim == 0
    t = np.atleast_1d(t)
    result = np.zeros_like(t)
    
    for i, th in enumerate(t):
        s_sum = 0.0
        for k in range(1, N+1):
            frac = k / N
            th_k = frac * th
            s_sum += np.sin(th_k)**2
        result[i] = s_sum / (N**2)
    
    return result[0] if scalar else result


def _H8_taylor(theta, N=10):
    t = np.asarray(theta, dtype=float)
    const = sum((k/N)**2 for k in range(1, N+1)) / (N**2)
    return np.full_like(t, const, dtype=float)


def H8(theta, N=10):
    return _safe_eval(lambda x: _H8_exact(x, N),
                      lambda x: _H8_taylor(x, N),
                      theta)


# ─────────────────────────────────────────────────────────────────────────────
# ANALYTICAL DERIVATIVES FOR H1 AND H2 (As requested in Phase 2)
# ─────────────────────────────────────────────────────────────────────────────

def _dH1_exact(theta):
    """Analytical derivative of H1 exact form."""
    t = np.asarray(theta, dtype=float)
    num = -2*t**3 - 24*t - 6*t**2 * np.sin(t) - 36*t * np.cos(t) + 60*np.sin(t)
    return num / t**6

def _dH1_taylor(theta):
    """Analytical derivative of H1 Taylor form (dH1/dθ = θ³/2160 - θ/84)."""
    t = np.asarray(theta, dtype=float)
    return t**3 / 2160 - t / 84

def dH1(theta):
    """Safe evaluation for dH1/dθ"""
    return _safe_eval(_dH1_exact, _dH1_taylor, theta)


def _dH2_exact(theta):
    """Analytical derivative of H2 exact form."""
    t = np.asarray(theta, dtype=float)
    num = -12*t - 8*t*np.cos(t) + 2*t*np.cos(2*t) + 24*np.sin(t) - 3*np.sin(2*t)
    return num / t**4

def _dH2_taylor(theta):
    """Analytical derivative of H2 Taylor form (dH2/dθ = -2θ³/21 + 2θ/5)."""
    t = np.asarray(theta, dtype=float)
    return -2*t**3 / 21 + 2*t / 5

def dH2(theta):
    """Safe evaluation for dH2/dθ"""
    return _safe_eval(_dH2_exact, _dH2_taylor, theta)


# ─────────────────────────────────────────────────────────────────────────────
# PARTIAL DERIVATIVES dH/dθ (used in Coriolis matrix C)
# ─────────────────────────────────────────────────────────────────────────────

def dH_dtheta(H_func, theta, eps=1e-6):
    """Central-difference numerical derivative for H3-H8."""
    return (H_func(theta + eps) - H_func(theta - eps)) / (2 * eps)

# Convenience wrappers for all eight
def get_all_H(theta):
    """Return [H1, H2, ..., H8] as array."""
    return np.array([H1(theta), H2(theta), H3(theta), H4(theta),
                     H5(theta), H6(theta), H7(theta), H8(theta)])

def get_all_dH(theta):
    """Return [dH1/dθ, dH2/dθ, ..., dH8/dθ] as array."""
    return np.array([
        dH1(theta),                 # Analytical (Taylor/Exact safe)
        dH2(theta),                 # Analytical (Taylor/Exact safe)
        dH_dtheta(H3, theta),       # Numerical
        dH_dtheta(H4, theta),       # Numerical
        dH_dtheta(H5, theta),       # Numerical
        dH_dtheta(H6, theta),       # Numerical
        dH_dtheta(H7, theta),       # Numerical
        dH_dtheta(H8, theta)        # Numerical
    ])
