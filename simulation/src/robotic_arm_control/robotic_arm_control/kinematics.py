"""Dependency-free (numpy only) kinematics of the arm, read directly from the URDF.

Used by the ROS nodes (IK for the pick-and-place) and by the offline analysis (gravity torques,
workspace, clearance), so that both use exactly the same model.  Cross-checked against pinocchio
in robotic_arm_analysis/cross_check.py.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

ARM_JOINTS = ["base_rotation_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_pitch_joint", "wrist_roll_joint", "tool_rotation_joint"]
JAW_JOINTS = ["left_jaw_joint", "right_jaw_joint"]
G = 9.81


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, angle):
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def rot_log(R):
    """rotation vector of R."""
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    th = math.acos(c)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if th < 1e-9:
        return w / 2
    if math.pi - th < 1e-6:                      # near 180 deg: use the diagonal
        a = np.sqrt(np.maximum((np.diag(R) + 1) / 2, 0))
        k = int(np.argmax(a))
        a = a * np.sign(R[k] + R[:, k] + 1e-12)
        a[k] = abs(a[k])
        return th * a / np.linalg.norm(a)
    return th / (2 * math.sin(th)) * w


def _T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


class ArmModel:
    def __init__(self, urdf_path=None, urdf_string=None):
        root = ET.parse(urdf_path).getroot() if urdf_string is None else ET.fromstring(urdf_string)
        f3 = lambda s: [float(v) for v in s.split()]
        self.links = {}
        for l in root.findall("link"):
            ine = l.find("inertial")
            if ine is None:
                self.links[l.get("name")] = dict(mass=0.0, com=np.zeros(3))
            else:
                self.links[l.get("name")] = dict(mass=float(ine.find("mass").get("value")),
                                                 com=np.array(f3(ine.find("origin").get("xyz"))))
        self.joints = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            a = j.find("axis")
            lim = j.find("limit")
            dyn = j.find("dynamics")
            ax = np.array(f3(a.get("xyz"))) if a is not None else np.array([0.0, 0, 1])
            self.joints[j.get("name")] = dict(
                type=j.get("type"), parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                T=_T(f3(o.get("xyz")), f3(o.get("rpy"))), axis=ax / np.linalg.norm(ax),
                lower=float(lim.get("lower")) if lim is not None else 0.0,
                upper=float(lim.get("upper")) if lim is not None else 0.0,
                effort=float(lim.get("effort")) if lim is not None else 0.0,
                velocity=float(lim.get("velocity")) if lim is not None else 0.0,
                damping=float(dyn.get("damping")) if dyn is not None else 0.0,
                friction=float(dyn.get("friction")) if dyn is not None else 0.0)
        self.children = {}
        for n, j in self.joints.items():
            self.children.setdefault(j["parent"], []).append(n)
        self.lower = np.array([self.joints[n]["lower"] for n in ARM_JOINTS])
        self.upper = np.array([self.joints[n]["upper"] for n in ARM_JOINTS])
        self.effort = np.array([self.joints[n]["effort"] for n in ARM_JOINTS])
        self.velocity = np.array([self.joints[n]["velocity"] for n in ARM_JOINTS])
        self.link_order = [self.joints[n]["child"] for n in ARM_JOINTS]
        # fast path: serial chain world -> tcp (fixed transforms, axes, lumped masses per chain link)
        Tw = self.joints["world_to_base"]["T"]
        self._c_T, self._c_K, self._c_K2, self._c_ax = [], [], [], []
        for i, n in enumerate(ARM_JOINTS):
            j = self.joints[n]
            self._c_T.append(Tw @ j["T"] if i == 0 else j["T"])
            a = j["axis"]
            K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            self._c_K.append(K)
            self._c_K2.append(K @ K)
            self._c_ax.append(a)
        self._c_tcp = self.joints["tcp_joint"]["T"]
        self._c_mass = []                      # per chain link: (mass, com in the link frame), jaws lumped in the tool link
        for ln in self.link_order:
            m, mc = self.links[ln]["mass"], self.links[ln]["mass"] * self.links[ln]["com"]
            for jn in self.children.get(ln, []):
                j = self.joints[jn]
                if jn not in ARM_JOINTS and self.links[j["child"]]["mass"] > 0:
                    c = j["T"][:3, :3] @ self.links[j["child"]]["com"] + j["T"][:3, 3]
                    m, mc = m + self.links[j["child"]]["mass"], mc + self.links[j["child"]]["mass"] * c
            self._c_mass.append((m, mc / m))

    def chain(self, q):
        """fast FK along the serial chain: returns (list of 6 link transforms, list of (origin, axis), tcp transform)."""
        T = np.eye(4)
        Ts, axes = [], []
        for i in range(6):
            T = T @ self._c_T[i]
            axes.append((T[:3, 3].copy(), T[:3, :3] @ self._c_ax[i]))
            M = np.eye(4)
            M[:3, :3] += math.sin(q[i]) * self._c_K[i] + (1 - math.cos(q[i])) * self._c_K2[i]
            T = T @ M
            Ts.append(T)
        return Ts, axes, T @ self._c_tcp

    # ------------------------------------------------------------------ forward kinematics
    def fk(self, q, jaw=0.0):
        """returns {link: 4x4 world transform}, {joint: (origin point, world axis)} for the six arm joints."""
        qd = dict(zip(ARM_JOINTS, q))
        T = {"world": np.eye(4)}
        ax = {}
        stack = ["world"]
        while stack:
            p = stack.pop()
            for jn in self.children.get(p, []):
                j = self.joints[jn]
                Tj = T[p] @ j["T"]
                if j["type"] == "revolute":
                    ax[jn] = (Tj[:3, 3].copy(), Tj[:3, :3] @ j["axis"])
                    M = np.eye(4)
                    M[:3, :3] = axis_angle(j["axis"], qd.get(jn, 0.0))
                    Tj = Tj @ M
                elif j["type"] == "prismatic":
                    M = np.eye(4)
                    M[:3, 3] = j["axis"] * jaw
                    Tj = Tj @ M
                T[j["child"]] = Tj
                stack.append(j["child"])
        return T, ax

    def tcp(self, q):
        return self.chain(q)[2]

    def jacobian(self, q):
        _, axes, Tt = self.chain(q)
        p = Tt[:3, 3]
        J = np.zeros((6, 6))
        for i, (o, z) in enumerate(axes):
            J[:3, i] = np.cross(z, p - o)
            J[3:, i] = z
        return J, Tt

    # ------------------------------------------------------------------ gravity torques
    def gravity_torques(self, q, payload=0.0):
        """actuator torque (N.m) needed at each arm joint to hold the pose against gravity
        (static torque of everything beyond the joint, payload at the tcp included)."""
        Ts, axes, Tt = self.chain(q)
        tau = np.zeros(6)
        msum, mc = payload, payload * Tt[:3, 3]            # running sum of distal mass and mass*com
        for i in range(5, -1, -1):
            m, c = self._c_mass[i]
            msum += m
            mc = mc + m * (Ts[i][:3, :3] @ c + Ts[i][:3, 3])
            o, z = axes[i]
            r = mc / msum - o
            tau[i] = G * msum * (z[0] * r[1] - z[1] * r[0])   # -z . (r x (0,0,-mg))
        return tau

    def _distal(self, link):
        out = [link]
        for jn in self.children.get(link, []):
            out += self._distal(self.joints[jn]["child"])
        return out

    # ------------------------------------------------------------------ inverse kinematics
    @staticmethod
    def tool_down_rotation(yaw):
        """orientation of the tcp frame for a vertical tool: tool axis (local -Y) down, jaws sliding along
        the horizontal direction `yaw` (local X)."""
        x = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        y = np.array([0.0, 0.0, 1.0])
        return np.column_stack([x, y, np.cross(x, y)])

    @staticmethod
    def tool_rotation(azimuth, jaw_yaw=0.0, tilt=0.0):
        """orientation of the tcp frame at a station seen at `azimuth` from the base:
        jaws slide along the horizontal direction azimuth + jaw_yaw (local X); the tool axis (local -Y)
        points down, tilted by `tilt` about the jaw axis (so the jaw faces stay parallel to the cube faces)."""
        psi = azimuth + jaw_yaw
        x = np.array([math.cos(psi), math.sin(psi), 0.0])
        h = np.cross([0.0, 0.0, 1.0], x)
        y = math.cos(tilt) * np.array([0.0, 0.0, 1.0]) - math.sin(tilt) * h
        return np.column_stack([x, y, np.cross(x, y)])

    def ik_tool(self, pos, azimuth, jaw_yaw, tilt, q0, seeds=()):
        """IK for the tilted tool; the jaws are symmetric, so (jaw_yaw + pi, -tilt) is the same grasp and is
        tried as well.  The solution closest to q0 is returned."""
        best = None
        for s in (q0,) + tuple(seeds):
            for jy, tl in ((jaw_yaw, tilt), (jaw_yaw + math.pi, -tilt)):
                q, ok = self.ik(pos, self.tool_rotation(azimuth, jy, tl), s)
                if ok:
                    d = np.linalg.norm(q - np.asarray(q0))
                    if best is None or d < best[0]:
                        best = (d, q)
        return (best[1], True) if best else (np.array(q0, dtype=float), False)

    def ik(self, pos, R, q0, iters=200, tol_p=2e-4, tol_r=2e-3, damping=0.02):
        """damped least-squares IK with joint-limit clamping.  returns (q, ok)."""
        q = np.array(q0, dtype=float)
        for _ in range(iters):
            J, T = self.jacobian(q)
            ep = np.asarray(pos) - T[:3, 3]
            er = T[:3, :3] @ rot_log(T[:3, :3].T @ R)
            if np.linalg.norm(ep) < tol_p and np.linalg.norm(er) < tol_r:
                return q, True
            e = np.concatenate([ep, 0.3 * er])
            J[3:] *= 0.3
            dq = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(6), e)
            s = np.max(np.abs(dq))
            if s > 0.25:
                dq *= 0.25 / s
            q = np.clip(q + dq, self.lower, self.upper)
        return q, False

    def ik_tool_down(self, pos, yaw, q0, seeds=()):
        """tool-down IK; the jaws are symmetric, so yaw and yaw+pi are tried, as well as extra seeds.
        The solution closest to q0 is returned."""
        best = None
        for s in (q0,) + tuple(seeds):
            for y in (yaw, yaw + math.pi):
                q, ok = self.ik(pos, self.tool_down_rotation(y), s)
                if ok:
                    d = np.linalg.norm(q - np.asarray(q0))
                    if best is None or d < best[0]:
                        best = (d, q)
        return (best[1], True) if best else (np.array(q0, dtype=float), False)
