"""Walking pattern generator for the Unitree G1 (23-dof model).

Pure numpy + mujoco (no ROS). The pieces are:

* ``G1Kinematics``  - numeric leg inverse kinematics on the real MJCF model,
  with the pelvis position adjusted so the *whole-body* centre of mass (CoM)
  lands on a requested (x, y) point.
* ``WalkPlan``      - footsteps + a linear-inverted-pendulum (LIPM) CoM
  reference built with the divergent-component-of-motion (DCM) method.
* ``WalkGenerator`` - combines both: ``joint_targets(t)`` returns the 12 leg
  joint angles for time ``t`` of the walking sequence.

Conventions: world x is forward, y is left, z is up. A "foot point" is the
centre of the foot's support polygon (ankle frame origin + FOOT_CENTER_X).
"""
import math

import mujoco
import numpy as np

GRAVITY = 9.81
# Support polygon centre of the foot, along x, relative to the ankle-roll
# frame origin (foot contact spheres span x = -0.05 ... +0.12).
FOOT_CENTER_X = 0.035
# Vertical distance from the ankle-roll frame origin down to the sole.
ANKLE_TO_SOLE = 0.035

LEG_JOINTS = {
    'left': [
        'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
        'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    ],
    'right': [
        'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
        'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    ],
}
FOOT_BODY = {'left': 'left_ankle_roll_link', 'right': 'right_ankle_roll_link'}


class G1Kinematics:
    """Leg IK + CoM queries on a private copy of the G1 model."""

    def __init__(self, mjcf_path):
        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.pelvis_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'pelvis')
        self.foot_id = {
            s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, FOOT_BODY[s])
            for s in LEG_JOINTS
        }
        self.qadr, self.dadr, self.lo, self.hi = {}, {}, {}, {}
        for side, names in LEG_JOINTS.items():
            ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in names]
            self.qadr[side] = np.array([m.jnt_qposadr[i] for i in ids])
            self.dadr[side] = np.array([m.jnt_dofadr[i] for i in ids])
            self.lo[side] = np.array([m.jnt_range[i][0] for i in ids])
            self.hi[side] = np.array([m.jnt_range[i][1] for i in ids])
        self._jacp = np.zeros((3, m.nv))
        self._jacr = np.zeros((3, m.nv))
        mujoco.mj_resetData(m, self.data)
        self.data.qpos[3] = 1.0
        self.set_pelvis([0.0, 0.0, 0.7])
        # Start IK from a bent-knee seed: the straight-leg (all zero) pose is
        # a kinematic singularity and makes the first IK step erratic.
        for side in LEG_JOINTS:
            self.set_leg(side, [-0.3, 0.0, 0.0, 0.6, -0.3, 0.0])

    # ----------------------------------------------------------------- state
    def set_pelvis(self, pos, pitch=0.0):
        """Place the pelvis at ``pos`` with a pitch (about world y) in rad."""
        self.data.qpos[0:3] = pos
        self.data.qpos[3:7] = [math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0]

    def get_leg(self, side):
        return self.data.qpos[self.qadr[side]].copy()

    def set_leg(self, side, q):
        self.data.qpos[self.qadr[side]] = q

    def com(self):
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)
        return self.data.subtree_com[self.pelvis_id].copy()

    # -------------------------------------------------------------------- IK
    def solve_leg(self, side, ankle_pos, iters=30, tol=1e-6, damping=1e-3,
                  max_step=0.2):
        """Move one leg so its ankle frame origin is at ``ankle_pos`` (world)
        with the foot level (world-aligned orientation). Returns the error
        norm after the last iteration."""
        m, d = self.model, self.data
        fid = self.foot_id[side]
        cols = self.dadr[side]
        q = self.data.qpos[self.qadr[side]].copy()
        err_norm = 0.0
        for _ in range(iters):
            mujoco.mj_kinematics(m, d)
            rot = d.xmat[fid].reshape(3, 3)
            err_p = np.asarray(ankle_pos) - d.xpos[fid]
            # level foot: target rotation is identity -> axis-angle error
            err_r = 0.5 * (np.cross(rot[:, 0], [1, 0, 0])
                           + np.cross(rot[:, 1], [0, 1, 0])
                           + np.cross(rot[:, 2], [0, 0, 1]))
            err = np.concatenate([err_p, err_r])
            err_norm = float(np.linalg.norm(err))
            if err_norm < tol:
                break
            mujoco.mj_jacBody(m, d, self._jacp, self._jacr, fid)
            jac = np.vstack([self._jacp[:, cols], self._jacr[:, cols]])
            dq = jac.T @ np.linalg.solve(jac @ jac.T + damping * np.eye(6), err)
            step = float(np.max(np.abs(dq)))
            if step > max_step:
                dq *= max_step / step
            q = np.clip(q + dq, self.lo[side], self.hi[side])
            self.data.qpos[self.qadr[side]] = q
        return err_norm

    def solve_pose(self, com_xy, pelvis_z, left_foot, right_foot,
                   outer_iters=4, pelvis_pitch=0.0):
        """Whole-body posture: feet (foot points, world xyz of the *sole
        centre*) fixed, pelvis height fixed, pelvis x/y iterated so the
        whole-body CoM x/y equals ``com_xy``.

        Returns (left_q, right_q, com, pelvis_pos)."""
        pel = self.data.qpos[0:3].copy()
        pel[2] = pelvis_z
        com = None
        for _ in range(outer_iters):
            self.set_pelvis(pel, pelvis_pitch)
            for side, foot in (('left', left_foot), ('right', right_foot)):
                ankle = np.array([foot[0] - FOOT_CENTER_X, foot[1],
                                  foot[2] + ANKLE_TO_SOLE])
                self.solve_leg(side, ankle)
            com = self.com()
            pel[0] += com_xy[0] - com[0]
            pel[1] += com_xy[1] - com[1]
        # final consistent solve at the last pelvis position
        self.set_pelvis(pel, pelvis_pitch)
        for side, foot in (('left', left_foot), ('right', right_foot)):
            ankle = np.array([foot[0] - FOOT_CENTER_X, foot[1],
                              foot[2] + ANKLE_TO_SOLE])
            self.solve_leg(side, ankle)
        com = self.com()
        return self.get_leg('left'), self.get_leg('right'), com, pel.copy()


class WalkPlan:
    """Footsteps and CoM reference for ``n_steps`` steps of a straight walk.

    The CoM follows a LIPM whose ZMP sits on the support foot for the whole
    step. Boundary conditions come from the divergent component of motion
    (DCM, xi = x + v/omega): a backward recursion from the final standing
    pose gives the DCM every step must start with, and the ZMP of the initial
    weight-shift phase is chosen so the DCM arrives there exactly.

    Step numbering: the first support foot is the left one, so step 1 swings
    the right foot; f[k] is the foot point landed by step k (f[0] = initial
    left, f[-1] = initial right).
    """

    def __init__(self, step_length=0.10, step_time=0.7, n_steps=8,
                 half_width=0.09, swing_height=0.04, double_support=0.25,
                 com_height=0.62, rest_time=1.0, shift_time=0.5,
                 hold_time=0.5, settle_time=1.5):
        self.s = step_length
        self.T = step_time
        self.N = int(n_steps)
        self.d = half_width
        self.h_sw = swing_height
        self.ds = double_support
        self.T_rest = rest_time
        self.T_shift = shift_time
        self.T_hold = hold_time
        self.Te = settle_time
        self.T0 = rest_time + shift_time + hold_time   # time step 1 starts
        self.Tc = math.sqrt(com_height / GRAVITY)
        if self.N < 2:
            raise ValueError('n_steps must be >= 2')

        # ---- footsteps (foot points, xy) ---------------------------------
        f = {-1: np.array([0.0, -self.d]), 0: np.array([0.0, self.d])}
        for k in range(1, self.N + 1):
            x = (k - 0.5) * self.s if k < self.N else f[k - 1][0]
            y = -self.d if k % 2 == 1 else self.d
            f[k] = np.array([x, y])
        self.f = f
        self.c0 = 0.5 * (f[-1] + f[0])
        self.c_final = 0.5 * (f[self.N - 1] + f[self.N])

        # ---- DCM boundary conditions (backward recursion) ---------------------
        decay = math.exp(-self.T / self.Tc)
        xi_end = {self.N: self.c_final.copy()}
        xi_start = {}
        for k in range(self.N, 0, -1):
            p = f[k - 1]
            xi_start[k] = p + (xi_end[k] - p) * decay
            xi_end[k - 1] = xi_start[k]
        # ---- initial weight shift onto the first support foot ----------------
        # rest at the centre -> ZMP nudged to the swing side pushes the DCM
        # towards the support foot -> ZMP on the support foot holds it there
        # until step 1. p_a is solved so DCM arrives at xi_start[1] exactly.
        p_sup = f[0]
        xi_b = p_sup + (xi_start[1] - p_sup) * math.exp(-self.T_hold / self.Tc)
        grow = math.exp(self.T_shift / self.Tc)
        self.zmp_shift = (xi_b - self.c0 * grow) / (1.0 - grow)
        self.xi_start = xi_start

        # ---- CoM segments: (t0, t1, zmp, x0, xi0) ----------------------------
        self.segments = []
        x0, xi0, t = self.c0.copy(), self.c0.copy(), 0.0
        seq = [(self.T_rest, self.c0, self.c0.copy()),
               (self.T_shift, self.zmp_shift, self.c0.copy()),
               (self.T_hold, p_sup, xi_b)]
        for k in range(1, self.N + 1):
            seq.append((self.T, f[k - 1], xi_start[k]))
        seq.append((self.Te, self.c_final, self.c_final.copy()))
        for dur, zmp, xi in seq:
            self.segments.append((t, t + dur, zmp, x0.copy(), xi.copy()))
            x0 = self._lipm(zmp, x0, xi, dur)
            t += dur
        self.duration = t

    # ---------------------------------------------------------------- CoM
    def _lipm(self, zmp, x0, xi0, tau):
        u = tau / self.Tc
        return zmp + (x0 - zmp) * math.exp(-u) + (xi0 - zmp) * math.sinh(u)

    def com_xy(self, t):
        t = min(max(t, 0.0), self.duration)
        for (t0, t1, zmp, x0, xi0) in self.segments:
            if t <= t1:
                return self._lipm(zmp, x0, xi0, t - t0)
        t0, t1, zmp, x0, xi0 = self.segments[-1]
        return self._lipm(zmp, x0, xi0, t1 - t0)

    def com_state(self, t):
        """Reference CoM position and velocity (xy) at time ``t``."""
        t = min(max(t, 0.0), self.duration)
        seg = self.segments[-1]
        for cand in self.segments:
            if t <= cand[1]:
                seg = cand
                break
        t0, t1, zmp, x0, xi0 = seg
        u = min(t - t0, t1 - t0) / self.Tc
        pos = zmp + (x0 - zmp) * math.exp(-u) + (xi0 - zmp) * math.sinh(u)
        vel = (-(x0 - zmp) * math.exp(-u) + (xi0 - zmp) * math.cosh(u)) / self.Tc
        return pos, vel

    def zmp_xy(self, t):
        for (t0, t1, zmp, _x0, _xi0) in self.segments:
            if t <= t1:
                return zmp
        return self.segments[-1][2]

    # -------------------------------------------------------------- feet
    def feet(self, t):
        """Return (left_xyz, right_xyz) foot points at time ``t``."""
        def xyz(p, z=0.0):
            return np.array([p[0], p[1], z])

        k = int(math.floor((t - self.T0) / self.T)) + 1
        if k < 1:
            return xyz(self.f[0]), xyz(self.f[-1])
        if k > self.N:
            return xyz(self.f[self.N if self.N % 2 == 0 else self.N - 1]), \
                xyz(self.f[self.N if self.N % 2 == 1 else self.N - 1])
        t_in = t - (self.T0 + (k - 1) * self.T)
        td = 0.5 * self.ds * self.T
        s = (t_in - td) / (self.T - 2 * td)
        a, b = self.f[k - 2], self.f[k]
        if s <= 0.0:
            swing = xyz(a)
        elif s >= 1.0:
            swing = xyz(b)
        else:
            sx = s - math.sin(2 * math.pi * s) / (2 * math.pi)
            swing = xyz(a + (b - a) * sx, self.h_sw * 0.5 * (1 - math.cos(2 * math.pi * s)))
        support = xyz(self.f[k - 1])
        if k % 2 == 1:      # right foot swings
            return support, swing
        return swing, support


class WalkGenerator:
    """Turns a ``WalkPlan`` into leg joint targets for any time t."""

    def __init__(self, mjcf_path, pelvis_height=0.70, **plan_kwargs):
        self.kin = G1Kinematics(mjcf_path)
        self.pelvis_height = pelvis_height
        plan_kwargs.setdefault('com_height', 0.0)
        # LIPM height = whole-body CoM height in the walking posture
        d = plan_kwargs.get('half_width', 0.09)
        _l, _r, com, _p = self.kin.solve_pose(
            [0, 0], pelvis_height, [0, d, 0], [0, -d, 0])
        if plan_kwargs['com_height'] <= 0.0:
            plan_kwargs['com_height'] = float(com[2])
        self.plan = WalkPlan(**plan_kwargs)
        self.duration = self.plan.duration

    def joint_targets(self, t):
        """Return (left_q[6], right_q[6], info dict) for time ``t``."""
        com_ref = self.plan.com_xy(t)
        left, right = self.plan.feet(t)
        lq, rq, com, pel = self.kin.solve_pose(
            com_ref, self.pelvis_height, left, right)
        return lq, rq, {'com_ref': com_ref, 'com': com, 'pelvis': pel,
                        'left': left, 'right': right,
                        'zmp': self.plan.zmp_xy(t)}
