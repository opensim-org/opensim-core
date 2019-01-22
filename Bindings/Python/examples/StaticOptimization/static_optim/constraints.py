"""
Constraints class.

ConstraintAccelerationTarget create a strict constraint for the optimization such as the generalized accelerations
computed from muscle activation equal the generalized accelerations from inverse kinematics.
"""
import numpy as np
from scipy.optimize import approx_fprime


class ConstraintAccelerationTarget:
    def __init__(self):
        pass

    def constraints(self, x, idx=None):
        """callback for calculating the constraints."""
        qddot_from_muscles = self.forward_dynamics(x)
        const = [
            self.actual_qddot[idx_q] - qddot_from_muscles.get(idx_q)
            for idx_q in range(qddot_from_muscles.size())
        ]
        if idx is not None:
            return const[idx]
        else:
            return const

    def jacobian(self, x):
        jac = np.ndarray([self.n_dof, self.n_actuators])
        for idof in range(self.n_dof):
            jac[idof, :] = approx_fprime(x, self.constraints, 1e-10, idof)
        return jac
