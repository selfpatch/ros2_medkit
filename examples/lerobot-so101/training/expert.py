# Copyright 2026 ahmedsleem109
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Closed-loop scripted pick-lift expert for so101-nexus MuJoCoPickLift-v1.

SCRIPTED, not learned: it reads the cube pose from the simulator (privileged
state) and uses IK. It only generates training demonstrations for ACT.

Unlike expert.py (whole path planned at reset), every action here is computed
from the arm's measured joints and the cube's current pose, and phases advance
on events (gripper at the cube, cube lifted), not on a clock. That lets the
recorder perturb the executed command (DART-style noise) while the dataset
stores the expert's corrective label, so the policy sees how to recover.

Phases: reach (joint interpolation to a centred pregrasp pose) -> descend (IK
onto the live cube) -> close -> lift -> hold. If the cube does not follow the
lift, it reopens, rises above the cube and tries again (up to 3 attempts).
"""

from __future__ import annotations

import mujoco
import numpy as np

from so101_nexus._pregrasp import PregraspSolver
from so101_nexus.config import SO101_JOINT_NAMES, SO101_TCP_SITE_NAME
from so101_nexus.kinematics import ee_ik_delta_q
import so101_nexus.mujoco  # noqa: F401  registers the envs

GRIPPER_OPEN = 1.3  # rad, the pregrasp solver's open target
GRIPPER_CLOSED = -0.17  # rad, lower actuator bound


def _smooth(t: float) -> float:
    t = min(max(t, 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


class _Centred:
    """Pregrasp draws: first candidate centred (no tilt, no offset), rest random fallback."""

    def __init__(self, rng: np.random.Generator):
        self.rng = rng

    def random(self, shape):
        d = self.rng.random(shape)
        d[0] = 0.5
        return d


class ClosedLoopExpert:
    def __init__(
        self,
        env,
        *,
        lift_dz: float = 0.10,
        t_reach: int = 60,
        t_descend: int = 40,
        t_close: int = 25,
        t_lift: int = 50,
        t_hold: int = 20,
        max_attempts: int = 3,
        rng: np.random.Generator | None = None,
    ):
        self.u = env.unwrapped
        self.model = self.u.model
        self.scratch = mujoco.MjData(self.model)
        self.site = self.model.site(SO101_TCP_SITE_NAME).id
        joints = [self.model.joint(n).id for n in SO101_JOINT_NAMES]
        self.qadr = self.model.jnt_qposadr[joints]
        self.dadr = self.model.jnt_dofadr[joints[:-1]]
        acts = [self.model.actuator(n).id for n in SO101_JOINT_NAMES]
        self.low = self.model.actuator_ctrlrange[acts, 0]
        self.high = self.model.actuator_ctrlrange[acts, 1]
        self.lift_dz = lift_dz
        self.t_reach, self.t_desc, self.t_close, self.t_lift, self.t_hold = (
            t_reach,
            t_descend,
            t_close,
            t_lift,
            t_hold,
        )
        self.max_attempts = max_attempts
        rng = rng or np.random.default_rng(0)
        slot = self.u._slots[self.u._target_slot_idx]
        self.q_start = self._q()
        self.q_pre = PregraspSolver(self.model).solve(
            self.u.data.qpos.copy(), slot, _Centred(rng).random((8, 5))
        )
        self.ok = self.q_pre is not None
        if self.ok:
            self.cube_geom = slot.geom_ids[0]
            self.q_pre = self._align_roll(self.q_pre)
            pre_pos, self.quat = self._fk(self.q_pre)
            self.hover = pre_pos[2] - self._cube()[2]
        self.phase, self.k, self.attempts, self.done = 'reach', 0, 1, False

    # -- helpers on a scratch copy, never touching the live sim --
    def _q(self) -> np.ndarray:
        return self.u.data.qpos[self.qadr].copy()

    def _cube(self) -> np.ndarray:
        return np.asarray(self.u._get_target_pose()[:3], dtype=np.float64).copy()

    def _fk(self, q: np.ndarray):
        d = self.scratch
        d.qpos[:] = self.u.data.qpos
        d.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.model, d)
        mujoco.mj_comPos(self.model, d)
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, d.site_xmat[self.site])
        return d.site_xpos[self.site].copy(), quat

    def _yaw_err(self, q: np.ndarray) -> float:
        """Cube yaw minus gripper yaw, wrapped to (-45, 45] deg (a cube is 90-deg symmetric)."""
        self._fk(q)
        g = self.scratch.site_xmat[self.site].reshape(3, 3)[:, 0]
        c = self.u.data.geom_xmat[self.cube_geom].reshape(3, 3)[:, 0]
        d = np.degrees(np.arctan2(c[1], c[0]) - np.arctan2(g[1], g[0]))
        return (d + 45.0) % 90.0 - 45.0

    def _align_roll(self, q: np.ndarray) -> np.ndarray:
        """Turn the wrist so the fingers face the cube's sides, not its corners."""
        d = np.deg2rad(self._yaw_err(q))
        cands = []
        for sign in (1.0, -1.0):
            c = q.copy()
            c[4] = np.clip(c[4] + sign * d, self.low[4], self.high[4])
            cands.append((abs(self._yaw_err(c)), c))
        return min(cands, key=lambda x: x[0])[1]

    def _ik(self, pos: np.ndarray, gripper: float, iters: int = 30) -> np.ndarray:
        """Joint target reaching `pos` with the pregrasp orientation, from the MEASURED joints."""
        q = self._q()
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        for _ in range(iters):
            cur_pos, cur_quat = self._fk(q)
            mujoco.mj_jacSite(self.model, self.scratch, jacp, jacr, self.site)
            jac = np.vstack([jacp[:, self.dadr], jacr[:, self.dadr]])
            q[:-1] += ee_ik_delta_q(jac, cur_pos, cur_quat, pos, self.quat)
            q = np.clip(q, self.low, self.high)
        q[-1] = gripper
        return q

    def _go(self, phase: str):
        self.phase, self.k = phase, 0

    def _retry(self):
        if self.attempts < self.max_attempts:
            self.attempts += 1
            self._go('retry')
        else:
            self.done = True

    # -- one corrective label per control step --
    def act(self) -> np.ndarray:
        """Return the expert's joint target (6,) for the current sim state."""
        k = self.k
        self.k += 1
        cube = self._cube()
        tcp, _ = self._fk(self._q())
        if self.phase == 'reach':
            s = _smooth(k / self.t_reach)
            q = (1 - s) * self.q_start + s * self.q_pre
            q[-1] = GRIPPER_OPEN
            if k + 1 >= self.t_reach:
                self._go('descend')
            return q
        if self.phase == 'descend':
            # Go down only while the gripper is over the cube; off to the side, stay high.
            lat = np.linalg.norm(tcp[:2] - cube[:2])
            high = np.clip((lat - 0.008) / 0.012, 0.0, 1.0)
            z = cube[2] + self.hover * max(1 - _smooth(k / self.t_desc), high)
            q = self._ik(np.array([cube[0], cube[1], z]), GRIPPER_OPEN)
            if k >= self.t_desc and np.linalg.norm(tcp - cube) < 0.006:
                self.grasp = cube.copy()
                self._go('close')
            elif k >= self.t_desc + 40:  # blocked (e.g. a finger on the cube): back off, retry
                self._retry()
            return q
        if self.phase == 'close':
            s = _smooth(k / self.t_close)
            q = self._ik(self.grasp, (1 - s) * GRIPPER_OPEN + s * GRIPPER_CLOSED)
            if k + 1 >= self.t_close:
                self.cube0 = cube.copy()
                self._go('lift')
            return q
        if self.phase == 'lift':
            s = _smooth(k / self.t_lift)
            q = self._ik(self.grasp + [0, 0, s * self.lift_dz], GRIPPER_CLOSED)
            rose = tcp[2] - self.grasp[2]
            if rose > 0.03 and cube[2] - self.cube0[2] < 0.5 * rose:
                self._retry()  # cube slipped
            elif k + 1 >= self.t_lift:
                self.hold_pos = self.grasp + [0, 0, self.lift_dz]
                self._go('hold')
            return q
        if self.phase == 'retry':  # reopen and rise above the live cube, then descend again
            s = _smooth(k / 20)
            above = np.array([cube[0], cube[1], cube[2] + self.hover])
            q = self._ik(above, (1 - s) * GRIPPER_CLOSED + s * GRIPPER_OPEN)
            if k + 1 >= 30:
                self._go('descend')
            return q
        # hold
        q = self._ik(self.hold_pos, GRIPPER_CLOSED)
        if cube[2] - self.cube0[2] < 0.5 * self.lift_dz:
            self._retry()  # dropped while holding
        elif k + 1 >= self.t_hold:
            self.done = True
        return q


class Perturber:
    """Noise added to the EXECUTED command only (the dataset keeps the clean label).

    Ornstein-Uhlenbeck noise on the 5 arm joints, strongest while reaching, small
    while closing/lifting, plus at most one larger shove (4-8 deg on shoulder/elbow/
    wrist for 15 steps) early in the descent, like the drift seen in the failed rollouts.
    A fraction of episodes is recorded without any noise.
    """

    SIGMA = {
        'reach': np.deg2rad(2.0),
        'descend': np.deg2rad(0.5),
        'retry': np.deg2rad(1.0),
        'close': np.deg2rad(0.3),
        'lift': np.deg2rad(0.3),
        'hold': np.deg2rad(0.3),
    }

    def __init__(self, rng: np.random.Generator, clean_frac: float = 0.25, shove_p: float = 0.5):
        self.rng = rng
        self.on = rng.random() >= clean_frac
        self.x = np.zeros(5)
        self.shove_at = int(rng.integers(0, 15)) if self.on and rng.random() < shove_p else -1
        self.shove = np.zeros(5)
        if self.shove_at >= 0:
            v = np.zeros(5)
            v[1:4] = rng.normal(size=3)
            self.shove = v / np.linalg.norm(v) * np.deg2rad(rng.uniform(4, 8))
        self.shove_left = 0

    def __call__(self, q: np.ndarray, phase: str, k_phase: int) -> np.ndarray:
        if not self.on:
            return q
        theta = 0.15
        sig = self.SIGMA[phase]
        self.x += -theta * self.x + sig * np.sqrt(2 * theta) * self.rng.normal(size=5)
        out = q.copy()
        out[:5] += self.x
        if phase == 'descend' and k_phase == self.shove_at:
            self.shove_left, self.shove_at = 15, -1
        if self.shove_left > 0:
            out[:5] += self.shove
            self.shove_left -= 1
        return out
