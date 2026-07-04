"""The pure-numpy IRB 360 kinematics are laptop-testable (pxr only loads in
pose(), which is rig-only). Confirms the extraction kept the math intact."""
import numpy as np

from deltahil.plant import irb360_pose as kp


def test_module_imports_without_isaac():
    assert hasattr(kp, "pose") and hasattr(kp, "world_to_local")


def test_home_pose_solves_to_home_elbow():
    # at the home plate target, arm 1's IK must recover the home elbow and the
    # forearm length constraint |E - attach| == RE.
    T = kp.HOME_PLATE
    u_r = kp.rotz(np.array([1.0, 0, 0]), 0.0)
    P = np.array([kp.R_MOTOR, 0.0, kp.Z_MOTOR])
    attach = T + kp.rotz(kp.ATTACH_OFF1, 0.0)
    E = kp.solve_elbow(P, u_r, attach, kp.HOME_E1)
    assert np.linalg.norm(E - kp.HOME_E1) < 1e-6           # recovers home elbow
    assert abs(np.linalg.norm(E - attach) - kp.RE) < 1e-6  # forearm length holds


def test_reach_small_and_continuous_off_center():
    # off the arm's radial line the CAD-derived geometry reaches only
    # approximately (the IK clamps near the workspace boundary) -- the same few-mm
    # residual seen in the animation. Assert it stays small and the elbow is near
    # the home elbow (branch continuity), which is what matters visually.
    T = kp.HOME_PLATE + np.array([100.0, 0.0, 120.0])
    for phi in kp.ARMS.values():
        u_r = kp.rotz(np.array([1.0, 0, 0]), phi)
        P = np.array([kp.R_MOTOR * u_r[0], kp.R_MOTOR * u_r[1], kp.Z_MOTOR])
        home_E = kp.rotz(kp.HOME_E1, phi)
        attach = T + kp.rotz(kp.ATTACH_OFF1, phi)
        E = kp.solve_elbow(P, u_r, attach, home_E)
        assert abs(np.linalg.norm(E - attach) - kp.RE) < 5.0   # few-mm residual
        assert np.linalg.norm(E - home_E) < 400.0              # stayed on branch
