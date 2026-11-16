#!/usr/bin/env python3
"""Cross-check of the numpy kinematics/gravity model against pinocchio on random poses (needs `pip install pin`)."""
import numpy as np

from .common import URDF, load


def main():
    import pinocchio as pin
    model, _, _ = load()
    pm = pin.buildModelFromUrdf(URDF)
    pd = pm.createData()
    rng = np.random.default_rng(0)
    e_g = e_p = 0.0
    for _ in range(500):
        q = rng.uniform(model.lower, model.upper)
        qp = np.zeros(pm.nq)
        qp[:6] = q
        e_g = max(e_g, np.abs(pin.computeGeneralizedGravity(pm, pd, qp)[:6] - model.gravity_torques(q)).max())
        pin.framesForwardKinematics(pm, pd, qp)
        e_p = max(e_p, np.abs(pd.oMf[pm.getFrameId("tcp")].homogeneous - model.tcp(q)).max())
    print(f"500 random poses: max gravity-torque difference {e_g:.2e} N.m, max tcp pose difference {e_p:.2e}")


if __name__ == "__main__":
    main()
