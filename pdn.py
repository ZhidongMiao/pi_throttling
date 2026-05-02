"""
pdn.py — Power Delivery Network (PDN) Model
=============================================
7th-order parallel IIR filter bank modeling the 3-mode PDN response.

Physical modes:
  Mode 1 (package): 3rd-order damped oscillator,  f1=32.14MHz, τ1=53.24ns
  Mode 2 (board):   3rd-order damped oscillator,  f2=5.69MHz,  τ2=98.64ns
  Mode 3 (VRM):     1st-order,                    τ3=1789ns

Transfer function: H(s) = H1(s) + H2(s) + H3(s), discretized via
matrix-exponential ZOH at Ts=0.625ns (1.6 GHz).

State:  7 floats (3+3+1),   7 MACs/cycle.
vs FIR: 480 floats,        480 MACs/cycle.

The IIR correctly models the infinite PDN tail; the FIR truncated at 480
cycles (~300ns), causing ~12mV systematic error for sustained loads.
"""

import numpy as np
from scipy.linalg import expm

# ── Global constants ─────────────────────────────────────────────────
FREQ_GHZ  = 1.6
DT_S      = 1.0 / (FREQ_GHZ * 1e9)
V0_MV     = 909.0
V_SIGNOFF = 675.0
V_MARGIN  = V0_MV - V_SIGNOFF  # 234 mV

# ── PDN fitted parameters ────────────────────────────────────────────
# Calibration: MULA×2 (tok=6) → 121mV droop at cy=114 (measured)
_A1 = 0.04647;  _f1 = 32.14e6;  _tau1 = 53.24e-9
_A2 = 0.11802;  _f2 = 5.69e6;   _tau2 = 98.64e-9
_A3 = 0.02598;  _tau3 = 1789.06e-9
_TOK_SCALE = 6.0
_SCALE_MV  = 1000.0 / _TOK_SCALE  # 166.67 mV/token


def _build_iir_coefficients():
    """Build discretized state-space matrices for the 3-mode parallel IIR."""
    alpha1 = 1.0/_tau1;  omega1 = 2*np.pi*_f1;  K1 = _SCALE_MV*_A1
    alpha2 = 1.0/_tau2;  omega2 = 2*np.pi*_f2;  K2 = _SCALE_MV*_A2
    alpha3 = 1.0/_tau3;                          K3 = _SCALE_MV*_A3

    def _ss_3rd(alpha, omega, K):
        """Controllable canonical form: H(s)=K·ω²·s/(s³+3αs²+(3α²+ω²)s+α(α²+ω²))"""
        w2 = omega*omega
        a0 = alpha*(alpha*alpha + w2)
        a1 = 3*alpha*alpha + w2
        a2 = 3*alpha
        Ac = np.array([[0., 1., 0.], [0., 0., 1.], [-a0, -a1, -a2]])
        Bc = np.array([[0.], [0.], [1.]])
        Cc = np.array([[0., K*w2, 0.]])
        Dc = np.array([[0.]])
        return Ac, Bc, Cc, Dc

    def _zoh(Ac, Bc, Cc, Dc, Ts):
        """ZOH discretization via matrix exponential [A B; 0 0]."""
        n = Ac.shape[0]
        M = np.zeros((n+1, n+1))
        M[:n, :n] = Ac;  M[:n, n:] = Bc
        eM = expm(M*Ts)
        return eM[:n, :n], eM[:n, n:].flatten(), Cc.flatten(), Dc.item()

    Ac1, Bc1, Cc1, Dc1 = _ss_3rd(alpha1, omega1, K1)
    Ac2, Bc2, Cc2, Dc2 = _ss_3rd(alpha2, omega2, K2)
    Ac3 = np.array([[-alpha3]]);  Bc3 = np.array([[K3*alpha3]])
    Cc3 = np.array([[1.]]);       Dc3 = np.array([[0.]])

    Ad1, Bd1, Cd1, Dd1 = _zoh(Ac1, Bc1, Cc1, Dc1, DT_S)
    Ad2, Bd2, Cd2, Dd2 = _zoh(Ac2, Bc2, Cc2, Dc2, DT_S)
    Ad3, Bd3, Cd3, Dd3 = _zoh(Ac3, Bc3, Cc3, Dc3, DT_S)

    return (Ad1, Bd1, Cd1, Dd1), (Ad2, Bd2, Cd2, Dd2), (Ad3, Bd3, Cd3, Dd3)


# Precomputed IIR coefficients (module-level, computed once at import)
(_AD1, _BD1, _CD1, _DD1), (_AD2, _BD2, _CD2, _DD2), (_AD3, _BD3, _CD3, _DD3) = \
    _build_iir_coefficients()


class PDNModel:
    """IIR PDN: V[k] = V0 - droop, where droop is 3-mode parallel IIR output.

    Input: raw token load (not delta).  State: 7 floats.  7 MACs/cycle.
    """
    def __init__(self):
        self.reset()

    def step(self, token: float) -> float:
        t = float(token)
        y1 = float(np.dot(_CD1, self._x1) + _DD1*t)
        self._x1 = np.dot(_AD1, self._x1) + _BD1*t
        y2 = float(np.dot(_CD2, self._x2) + _DD2*t)
        self._x2 = np.dot(_AD2, self._x2) + _BD2*t
        y3 = float(np.dot(_CD3, self._x3) + _DD3*t)
        self._x3 = np.dot(_AD3, self._x3) + _BD3*t
        return float(np.clip(V0_MV - (y1 + y2 + y3), 400.0, V0_MV + 80.0))

    def reset(self):
        self._x1 = np.zeros(3);  self._x2 = np.zeros(3);  self._x3 = np.zeros(1)


class PDNObserver:
    """Digital twin of PDNModel — same IIR, returns unclipped estimate.

    Enables voltage-feedback control without hardware voltage ADC.
    """
    def __init__(self):
        self.voltage = V0_MV
        self.reset()

    def step(self, token: float) -> float:
        t = float(token)
        y1 = float(np.dot(_CD1, self._x1) + _DD1*t)
        self._x1 = np.dot(_AD1, self._x1) + _BD1*t
        y2 = float(np.dot(_CD2, self._x2) + _DD2*t)
        self._x2 = np.dot(_AD2, self._x2) + _BD2*t
        y3 = float(np.dot(_CD3, self._x3) + _DD3*t)
        self._x3 = np.dot(_AD3, self._x3) + _BD3*t
        self.voltage = V0_MV - (y1 + y2 + y3)
        return self.voltage

    def reset(self):
        self._x1 = np.zeros(3);  self._x2 = np.zeros(3);  self._x3 = np.zeros(1)
        self.voltage = V0_MV
