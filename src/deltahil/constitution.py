"""The session constitution, as data. Every module cites these by number.

Fixed for the project by explicit sign-off. If the build exposes a principle we
missed or got wrong, STOP and re-confirm rather than pressing on (protocol
step 4).
"""

PRINCIPLES = {
    "P1": "Real-time closed loop: the PLC free-runs on its own oscillator "
          "(un-pausable); any latency the sim adds to the loop is physically "
          "indistinguishable from real lag.",
    "P2": "I/O contract is the only channel: the PLC acts solely on its sampled "
          "tag map; unsensed plant state changes outcomes but never control.",
    "P3": "A pick is a physical coincidence: succeeds iff pose<tol AND grip in "
          "the timing window AND force/friction suffices -- jointly; the solver "
          "must resolve contact faithfully enough that this is determined.",
    "P4": "Parallel closed-chain kinematics: the Delta's loop closure must be "
          "honored (PhysX articulation != kinematic tree -> guide-joint rigging).",
    "P5": "Calibration corrects bias, not variance: it drives the identifiable "
          "systematic pose error toward zero but cannot touch the stochastic "
          "part; residual noise floors achievable reliability.",
    "P6": "Reproducibility is bounded by the live loop: no bit-exact replay with "
          "a free-running PLC; evals must be statistical, not single-trace.",
    "P7": "HIL value is conditional: worth it only if the program under test is "
          "the real deployed one and the sim reproduces faults rarer/costlier/"
          "more dangerous to trigger physically; $0 core.",
}

REFINEMENTS = {
    "A": "Two-tier I/O: FAST tier (EtherCAT/EtherNet-IP) for axis+sensor I/O "
         "under the eval-5 jitter bound; SLOW tier (OPC UA) for supervisory "
         "tags, exempt from it.",
}

# Which evals each module is accountable to.
EVAL_PROVENANCE = {
    "eval1_ik_error": ("P3", "P4", "rig: 0.5 mm IK error on rigged Delta"),
    "eval5_latency": ("P1", "A", "rig: <10 ms round-trip, sigma<1 ms jitter, FAST tier"),
    "eval10_calibration": ("P5", "P3", "desk: bias<0.5 mm AND success<=noise ceiling"),
}
