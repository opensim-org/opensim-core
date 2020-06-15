import numpy as np
import pylab as pl

a = 5e5 # N/m^3
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




depth = np.linspace(-0.01, 0.01, 1000)

fig = pl.figure()
ax = fig.add_subplot(3, 1, 1)
for xdot in np.array([-5, -2.5, -0.5, -0.25, 0, 0.25, 0.5, 2.5, 5])[::-1]:
    pl.plot(depth, normal_force(depth, xdot),
            label='depth rate=%.1f' % xdot)
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

#pl.show()
fig.savefig('contact_models.pdf')



fig = pl.figure(figsize=(4, 4))
ax = fig.add_subplot(2, 1, 1)
depth = np.linspace(-0.005, 0.005, 1000)
pl.plot(depth, normal_force(depth, 0))
ax.set_xlabel('depth (m)')
ax.set_title('normal force (N)', fontsize=8)
ax.spines['left'].set_position('zero')
ax.spines['bottom'].set_position('zero')
ax.spines['right'].set_color('none')
ax.spines['top'].set_color('none')
ax.xaxis.set_label_coords(0.75, -0.04)

ax = fig.add_subplot(2, 1, 2)
sliding_velocity = np.linspace(-1, 1, 1000)
pl.plot(sliding_velocity,
        friction_force_AckermannVanDenBogert2010(1, sliding_velocity),
        label="Ackermann and van den Bogert 2010")
ax.set_title('friction force (-)', fontsize=8)
ax.set_xlabel('slip speed (m/s)')
ax.set_xticks([-1, -0.5, 0, 0.5, 1.0])
ax.xaxis.set_label_coords(0.75, 0.35)
ax.spines['left'].set_position('zero')
ax.spines['bottom'].set_position('zero')
ax.spines['right'].set_visible(False)
ax.spines['top'].set_visible(False)

fig.tight_layout()

fig.savefig('AckermannVanDenBogert2010_normal_and_friction.png', dpi=300)





# Simulate 2d bouncing ball with friction
# ---------------------------------------
import scipy.integrate
g = 9.81
mass = 50.0
a = 5e7
b = 1.0
stiffness_fictitious = 1.0
coefficient_of_friction = 1.0
velocity_scaling_factor = 0.05
def dynamics(state, t):
    x = state[0]
    y = state[1]
    vx = state[2]
    vy = state[3]

    ground_height = 0.0
    depth = ground_height - y
    depth_pos = np.maximum(0, depth)
    depth_rate = -vy
    #if depth > 0:
    #    physical_normal_force = (
    #        # TODO np.maximum(-fstiffness, fdissipation)
    #        # https://simtk-confluence.stanford.edu/display/OpenSim/Dynamics
    #        # +Theory+and+Publications?preview=/3376330/3737531/Sherman-2011-SethDelp-Simbody-ProcediaIUTAM-v2-p241.pdf
    #        np.maximum(0, a * depth_pos**3 * (1 + b * depth_rate))
    #    )
    #else:
    #    physical_normal_force = 0
    normal_force_stiffness = a * depth_pos**3
    normal_force_dissipation = np.maximum(
        b * normal_force_stiffness * depth_rate,
        -normal_force_stiffness)
    physical_normal_force = normal_force_stiffness + normal_force_dissipation
    normal_force = physical_normal_force + stiffness_fictitious * depth
    dvydt = -g  + normal_force / mass

    z0 = np.exp(-vx / velocity_scaling_factor)
    print("DEBUG", t, normal_force, vx, z0)
    friction_force = ((1 - z0) / (1 + z0) * coefficient_of_friction *
                      #physical_normal_force)
                      normal_force)
    dvxdt = -friction_force / mass;
    return np.array([vx, vy, dvxdt, dvydt])

x0 = 0
y0 = 1
vx0 = 0.1
vy0 = 0
initial_state = [x0, y0, vx0, vy0]
N = 100
tf = 1.25
time = np.linspace(0, 1.25, N)
solution = scipy.integrate.odeint(dynamics, initial_state, time)

state_names = ['x', 'y', 'vx', 'vy']

fig = pl.figure(figsize=(4, 10))
for i in range(4):
    ax = fig.add_subplot(4, 1, i + 1)
    ax.plot(time, solution[:, i])
    ax.set_title(state_names[i])

pl.show()












