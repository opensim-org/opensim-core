import numpy as np
import pylab as pl

a = 5e7 # N/m^3
dissipation_coefficient = 1.0 # s/m
b = dissipation_coefficient
# See Simbody Theory Manual about when the std::max() is necessary.
def normal_force(x, xdot=0):
    # TODO explain why the inner max() is necessary; it's b/c of the velocity
    # dependence.
    contact_model = np.maximum(0, a * np.maximum(0, x)**3 * (1 + b * xdot))
    adjustment_for_numerics = 1.0 * x
    return contact_model + adjustment_for_numerics
    #return np.maximum(0, a * x**3 * (1 + b * xdot))

coefficient_of_friction = 1.0
def friction_force_AckermannVanDenBogert2010(fnormal, sliding_velocity):
    """Far from sliding_velocity=0, the force is a constant -mu*Fn for
    sliding_velocity < 0 and mu*Fn for sliding_velocity > 0, and there's a
    smooth transition between the two near sliding_velocity=0.
    """
    velocity_scaling_factor = 0.05
    exp = np.exp(-sliding_velocity / velocity_scaling_factor)
    return (1 - exp) / (1 + exp) * coefficient_of_friction * fnormal

# Simbody uses a model from Michael Hollars (see Doxygen):
# f = fn*[min(vs/vt,1)*(ud+2(us-ud)/(1+(vs/vt)^2))+uv*vs]
# u = 2*u1*u2/(u1+u2)
# Michael Hollars worked with Sherm at Protein Mechanics.
# https://simtk.org/api_docs/simbody/3.5/classSimTK_1_1HuntCrossleyForce.html
def friction_force_Simbody(fnormal, sliding_velocity):
    # TODO edit to be symmetric.
    transition_velocity = 0.01
    dynamic_coefficient_of_friction = coefficient_of_friction
    ud = dynamic_coefficient_of_friction
    static_coefficient_of_friction = coefficient_of_friction
    us = static_coefficient_of_friction
    viscous_coefficient_of_friction = coefficient_of_friction
    uv = viscous_coefficient_of_friction
    norm_velocity = sliding_velocity/transition_velocity
    return fnormal * (np.minimum(norm_velocity, 1) *
                      (ud + 2 *(us - ud) / (1 + (norm_velocity)**2)) +
                      uv * sliding_velocity)

# Model with Stribeck, Coulomb, and viscous friction.
# https://www.mathworks.com/help/physmod/simscape/ref/translationalfriction.html
def friction_force_MATLAB(fnormal, sliding_velocity):
    coulomb_friction_force = coefficient_of_friction * fnormal
    breakaway_friction_force = 1.1 * coulomb_friction_force
    breakaway_velocity = 0.01
    stribek_velocity_threshold = np.sqrt(2) * breakaway_velocity
    coulomb_velocity_threshold = breakaway_velocity / 10.0
    norm_velocity = sliding_velocity / stribek_velocity_threshold
    return np.sqrt(2 * np.e) * (breakaway_friction_force -
                                coulomb_friction_force) *\
           np.exp(-norm_velocity**2) * norm_velocity + \
           coulomb_friction_force * np.tanh(
               sliding_velocity/coulomb_velocity_threshold) + \
           coefficient_of_friction * sliding_velocity




penetration = np.linspace(-0.01, 0.01, 1000)

fig = pl.figure()
ax = fig.add_subplot(3, 1, 1)
for xdot in np.array([-5, -2.5, -0.5, -0.25, 0, 0.25, 0.5, 2.5, 5])[::-1]:
    pl.plot(penetration, normal_force(penetration, xdot),
            label='penetration rate=%.1f' % xdot)
pl.legend()
ax = fig.add_subplot(3, 1, 2)
sliding_velocity = np.linspace(-5, 5, 1000)
pl.plot(sliding_velocity,
        friction_force_AckermannVanDenBogert2010(1, sliding_velocity),
        label="Ackermann and van den Bogert 2010")
pl.plot(sliding_velocity,
        friction_force_Simbody(1, sliding_velocity),
        label="Simbody")
pl.plot(sliding_velocity,
        friction_force_MATLAB(1, sliding_velocity),
        label="MATLAB")

# TODO should we use abs(sliding_velocity)?
ax = fig.add_subplot(3, 1, 3)
sliding_velocity = np.linspace(0, 0.5, 1000)
pl.plot(sliding_velocity,
        friction_force_AckermannVanDenBogert2010(1, sliding_velocity),
        label="Ackermann and van den Bogert 2010")
pl.plot(sliding_velocity,
        friction_force_Simbody(1, sliding_velocity),
        label="Simbody")
pl.plot(sliding_velocity,
        friction_force_MATLAB(1, sliding_velocity),
        label="MATLAB")
pl.legend()
pl.show()

fig.savefig('contact_models.pdf')
