"""Point-based hand-eye / frame calibration.

Constitutional role
-------------------
Implements principle **P5**: *calibration corrects bias, not variance.* The
frame offset between the vision system and the robot base is a **systematic,
identifiable** rigid transform. Point registration (Kabsch/Umeyama) recovers it
exactly in the noise-free limit. What it provably *cannot* do is remove the
per-sample stochastic error -- that term is unidentifiable and sets the
reliability floor that eval 10 forces us to report honestly.

Relationship to AX = XB
-----------------------
The classical hand-eye problem (Tsai-Lenz, Park-Martin) solves ``AX = XB`` for a
camera-to-gripper transform from a set of relative motions. When you instead have
a calibration fixture with **known** correspondences -- true points ``P`` and the
vision system's reported points ``Q`` -- the problem collapses to rigid point
registration, whose closed-form SVD solution (Kabsch) is the correct and minimal
estimator. We use that form because it is desk-verifiable without a robot: it is
the honest core of P5, not a shortcut around it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FrameTransform:
    """Rigid transform q = R @ p + t (millimetres)."""

    R: np.ndarray  # (3, 3) rotation
    t: np.ndarray  # (3,) translation, mm

    def apply(self, p: np.ndarray) -> np.ndarray:
        return p @ self.R.T + self.t

    def invert_point(self, q: np.ndarray) -> np.ndarray:
        """Recover the source point p from a transformed point q."""
        return (q - self.t) @ self.R

    @staticmethod
    def identity() -> "FrameTransform":
        return FrameTransform(np.eye(3), np.zeros(3))


def kabsch(P: np.ndarray, Q: np.ndarray) -> FrameTransform:
    """Least-squares rigid transform mapping P onto Q (both N x 3).

    Returns the (R, t) that minimises ||R @ P_i + t - Q_i||^2. This is the
    maximum-likelihood estimate of the systematic frame offset under isotropic
    Gaussian noise -- exactly the bias term P5 says is identifiable.
    """
    P = np.asarray(P, float)
    Q = np.asarray(Q, float)
    if P.shape != Q.shape or P.ndim != 2 or P.shape[1] != 3:
        raise ValueError("P and Q must both be (N, 3)")
    cP, cQ = P.mean(0), Q.mean(0)
    H = (P - cP).T @ (Q - cQ)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])  # reflection guard -> proper rotation
    R = Vt.T @ D @ U.T
    t = cQ - R @ cP
    return FrameTransform(R, t)


def noise_reliability_ceiling(sigma: float, tol: float, dof: int = 3) -> float:
    """P(||epsilon|| < tol) for isotropic Gaussian noise with std ``sigma``.

    This is the hard ceiling on post-calibration success. Any measured success
    rate *above* this is physically impossible and, per eval 10, a FAIL -- it
    would mean calibration beat variance.
    """
    if sigma <= 0:
        return 1.0
    x = (tol / sigma) ** 2  # chi-square_dof variate at the boundary
    if dof == 3:  # closed form, no scipy dependency
        return math.erf(math.sqrt(x / 2)) - math.sqrt(2 * x / math.pi) * math.exp(-x / 2)
    return _chi2_cdf(x, dof)


def _chi2_cdf(x: float, k: int) -> float:
    """Regularised lower incomplete gamma P(k/2, x/2) via a series expansion."""
    a, x = k / 2.0, x / 2.0
    if x <= 0:
        return 0.0
    term = 1.0 / a
    total = term
    n = 0
    while abs(term) > 1e-12 * abs(total) and n < 1000:
        n += 1
        term *= x / (a + n)
        total += term
    return total * math.exp(-x + a * math.log(x) - math.lgamma(a))


@dataclass
class CalibrationResult:
    transform: FrameTransform          # estimated vision->base offset
    residual_bias_mm: float            # mean ||corrected - true|| systematic component
    residual_rms_mm: float             # rms of corrected error (approaches noise floor)
    n_samples: int


def calibrate(true_points: np.ndarray, reported_points: np.ndarray) -> CalibrationResult:
    """Estimate the frame offset and report the residual it leaves behind.

    ``residual_bias_mm`` is the norm of the *mean* corrected error -- the part
    P5 says we can drive toward zero. The spread around it is the irreducible
    noise, surfaced as ``residual_rms_mm`` so it can never be hidden.
    """
    true_points = np.asarray(true_points, float)
    reported_points = np.asarray(reported_points, float)
    tf = kabsch(true_points, reported_points)
    corrected = tf.invert_point(reported_points)
    err = corrected - true_points
    residual_bias = float(np.linalg.norm(err.mean(0)))
    residual_rms = float(np.sqrt((err ** 2).sum(1).mean()))
    return CalibrationResult(tf, residual_bias, residual_rms, len(true_points))
