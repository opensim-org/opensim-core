import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['text.usetex'] = True


class ExponentialCoordinateForce:
    def __init__(self, coordinate_limits, limit_forces, shape_factors):
        self.coordinate_limits = coordinate_limits
        self.limit_forces = limit_forces
        self.shape_factors = shape_factors

    def calc_force(self, q):
        return self.limit_forces[0]*np.exp(-self.shape_factors[0]*(q - self.coordinate_limits[0])) + \
              -self.limit_forces[1]*np.exp(self.shape_factors[1]*(q - self.coordinate_limits[1]))

q = np.linspace(-1.0, 1.0, 1000)

coordinate_limits = np.array([-0.2, 0.3])
limit_forces = np.array([50.0, 20.0])
shape_factors = np.array([75.0, 25.0])
force = ExponentialCoordinateForce(coordinate_limits, limit_forces, shape_factors)
force_values = force.calc_force(q)

plt.figure(figsize=(8, 4.5))

plt.plot(q, force_values, color='k', lw=3)

# plot vertical lines at theta and phi
plt.axvline(x=coordinate_limits[0], color='r', linestyle='--', zorder=0)
plt.axvline(x=coordinate_limits[1], color='r', linestyle='--', zorder=0)
plt.axvline(x=0, color='k', linestyle='--', lw=0.5)
plt.axhline(y=0, color='k', linestyle='--', lw=0.5)
plt.plot(coordinate_limits[0], limit_forces[0], 'ko', ms=8)
plt.plot(coordinate_limits[1], -limit_forces[1], 'ko', ms=8)

plt.text(0.9*coordinate_limits[0], 0.95*limit_forces[0],
        f'({coordinate_limits[0]:.1f}, {limit_forces[0]})', color='k', fontsize=14)
plt.text(0.4*coordinate_limits[1], -1.1*limit_forces[1],
        f'({coordinate_limits[1]:.1f}, {-limit_forces[1]})', color='k', fontsize=14)

plt.text(2.0*coordinate_limits[0], 1.3*limit_forces[0],
        r'$50 e^{-75(x+0.2)}$', color='k', fontsize=15)
plt.text(1.2*coordinate_limits[1], -2.5*limit_forces[1],
        r'$-20 e^{25(x-0.3)}$', color='k', fontsize=15)

plt.xlim(-0.5, 0.6)
plt.ylim(-100, 100)
plt.xlabel('coordinate value (rad or m)', fontsize=14)
plt.ylabel('limit force (N or Nm)', fontsize=14)

plt.tight_layout()
plt.savefig('exponential_coordinate_limit_force.png', dpi=500)
plt.show()
