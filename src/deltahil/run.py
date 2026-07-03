"""End-to-end demo + self-score. Run: ``python -m deltahil.run``.

Closes the loop over the mock PLC + mock plant, shows calibration lifting success
from ~0 to the noise ceiling, injects a challenge (jams + misfeeds) to show what
calibration cannot fix (P5), and self-scores eval 10 per the protocol.
"""
from __future__ import annotations

from .constitution import PRINCIPLES
from .evals import run_eval10
from .scenario import Scenario, default_offset
from .telemetry import calibration_offset, run_closed_loop


def _bar(x, width=28):
    n = int(round(x * width))
    return "#" * n + "-" * (width - n)


def main():
    print("=" * 66)
    print("Delta HIL -- closed-loop pick-and-place, headless mock")
    print("=" * 66)

    # ---- Closed loop, translation-bias offset (vec3 slow-tier correction) ----
    demo = Scenario(vision_offset=default_offset(deg=0.0), pose_sigma_mm=0.15,
                    seed=7)
    off, _ = calibration_offset(demo, n_calib=25)

    uncal = run_closed_loop(demo, n_picks=200)
    cal = run_closed_loop(demo, n_picks=200, calib_offset_xyz=off)
    print("\nClosed loop (P1/P2) -- PLC sees only sensor tags, plant only commands")
    print(f"  uncalibrated success  {uncal.success_rate:5.1%}  |{_bar(uncal.success_rate)}|")
    print(f"  calibrated   success  {cal.success_rate:5.1%}  |{_bar(cal.success_rate)}|")
    print(f"  fast-tier scan  mean {cal.mean_latency_ms:.3f} ms  "
          f"jitter {cal.jitter_ms:.3f} ms  (illustrative; eval 5 is rig-only)")

    # ---- Challenge: faults calibration cannot remove (P5) ----
    hard = Scenario(vision_offset=default_offset(deg=0.0), pose_sigma_mm=0.15,
                    jam_rate=0.08, misfeed_rate=0.05, seed=11)
    off_h, _ = calibration_offset(hard, n_calib=25)
    chal = run_closed_loop(hard, n_picks=200, calib_offset_xyz=off_h)
    print("\nChallenge (P5) -- calibrated, but 8% jams + 5% misfeeds injected")
    print(f"  calibrated   success  {chal.success_rate:5.1%}  |{_bar(chal.success_rate)}|")
    print("  -> capped below the clean ceiling: calibration corrects bias, not "
          "these stochastic faults.")

    # ---- Eval 10 self-score (desk-verifiable, full transform incl. rotation) ----
    e10 = run_eval10(Scenario(vision_offset=default_offset(deg=0.4),
                              pose_sigma_mm=0.15, seed=3))
    print("\n" + "-" * 66)
    print("SELF-SCORE (protocol step 4)")
    print("-" * 66)
    print(f"Eval 10  calibration  [P5, P3]  -- N={e10.n_calib}, tol={e10.tol_mm} mm")
    print(f"  (a) residual bias        {e10.residual_bias_mm:.4f} mm  "
          f"< {e10.tol_mm}  -> {'PASS' if e10.bias_ok else 'FAIL'}")
    print(f"  (b) calibrated success   {e10.calibrated_success:.3f}  vs ceiling "
          f"{e10.ceiling:.3f}  (not above) -> {'PASS' if e10.ceiling_ok else 'FAIL'}")
    print(f"      uncalibrated success {e10.uncalibrated_success:.3f}  "
          f"-> improves: {'PASS' if e10.improves else 'FAIL'}")
    print(f"  EVAL 10: {'PASS' if e10.passed else 'FAIL'}")
    print("\nRig-verifiable only (cannot self-score at the desk):")
    print("  Eval 1  0.5 mm IK error on the rigged PhysX Delta  [P3, P4]")
    print("  Eval 5  <10 ms round-trip, sigma<1 ms jitter, FAST tier  [P1, A]")
    print("  Evals 3/9  RTF>=1.0 and >=30 FPS on your Isaac + GPU  [P4, P7]")
    print("=" * 66)


if __name__ == "__main__":
    main()
