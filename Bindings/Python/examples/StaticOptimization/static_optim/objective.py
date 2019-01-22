"""
Objectives class.

ObjMinimizeActivation minimizes the absolute activation raised at the activation_exponent (2 as default).
"""
import numpy as np
import opensim as osim


class ObjMinimizeActivation:
    def __init__(self, activation_exponent=2):
        self.activation_exponent = activation_exponent
        self.muscles_actuators = self.model.getMuscles()
        self.n_muscles = self.muscles_actuators.getSize()

    def objective(self, x):
        """callback for calculating the objective."""
        return np.power(np.abs(x), self.activation_exponent).sum()

    def gradient(self, x):
        """callback for calculating the gradient."""
        return self.activation_exponent * np.power(
            np.abs(x), self.activation_exponent - 1
        )

    def get_bounds(self):
        forces = self.model.getForceSet()
        activation = {"min": [], "max": []}
        for i in range(forces.getSize()):
            f = osim.CoordinateActuator.safeDownCast(forces.get(i))
            if f:
                activation["min"].append(f.get_min_control())
                activation["max"].append(f.get_max_control())
            m = osim.Muscle.safeDownCast(forces.get(i))
            if m:
                activation["min"].append(m.get_min_control())
                activation["max"].append(m.get_max_control())
        return activation["min"], activation["max"]
