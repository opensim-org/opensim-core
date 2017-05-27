/* Comments from https://github.com/opensim-org/opensim-core/pull/663 that have
not yet been addressed:

@chrisdembia:
I like that the two are pretty much exactly the same. I wonder if it makes sense
though to _not_ have rigid contact and compliant contact be interchangeable
(i.e. have the same interface). One is a force element while the other is a
constraint. For example, I don't know how RigidContactForce would implement
`Force::calcForce()`.  Or maybe they can have the same interface but not both be
forces.

Would we also have a RigidCoordinateLimitForce to go along with
CoordinateLimitForce? Or would we have JointStopConstraint? Should all
unilateral constraints appear in the same inheritance hierarchy, or can we split
up things like joint stops (put in constraints) and sphere-plane contact (put in
forces)?

@sherm1:
That's a very close match for OpenSim's current HuntCrossley stuff, but that is
obsolete and I hope we can change to a better model in 4.0. The problem is that
it associates contact parameters with the force element rather than with the
contact surfaces. That was replaced in Simbody a long time ago with a better
approach where contact properties are associated with ContactSurface objects
fixed to bodies. Those are then combined pairwise when they are in contact to
compute the effective properties for a particular impact or contact. Then forces
get applied when pairs bump, but there is no separate force element.

A ContactSurface is a pair: geometry+material properties. The material
properties can be compliant or rigid or both. Then conceivably it is just a
request to the solver whether to use compliant or rigid contact; probably we'll
want to use both in the same simulations though.

That said, there are some simpler cases that it would be useful to try first. My
first cut at rigid contact in Simbody doesn't have the fancy broad phase
determination of which pairs of n things are in contact. At first I'll just have
pairwise contact elements: joint stops, ropes, and point-ground contacts where
each point has its own element. In those cases the contact element will be the
keeper of the contact parameters. It would be cool if those could also support
either compliant or rigid modeling for apples-to-apples comparisons; I don't
know how that would look in OpenSim. There are advantages to these pairwise
elements and I think we would want to keep them even once we have the nXn stuff
working.

Another tricky issue is how we should get output from these things. They will
generate forces when in contact but also very substantial impulses during impact
that ought to be reported somehow. Would that be in scope for pseudocode?

@sherm1:
> Would we also have a RigidCoordinateLimitForce to go along with
CoordinateLimitForce? Or would we have JointStopConstraint?

I don't think we should have any of these. Instead we should have a JointStop
(or CoordinateLimit) whose job is to keep a coordinate in a given range. Then we
should be able to control which model (compliant vs. rigid) at run time, just as
we control accuracy now. That is, we don't have "AccurateCoordinateLimitForce"
and "SloppyCoordinateLimitForce"; we just consider that something the solver
controls. Similarly, switching between compliant and rigid models is an "outer
loop" of accuracy control where we choose accurate vs. fast at the modeling
level.

@sherm1:
I'm hoping we can leave the old H&CForce contact system in place for backwards
compatibility in 4.0. I don't think it prevents us from adding a new contact
scheme. But I don't want to model the rigid contact after the obsolete H&CForce
stuff.

@sherm1:
I want to discuss this more but I think it would be easier in person and with a
whiteboard first. Hopefully we'll have some time at Thursday's meeting.

For example: most existing models provide a range for joint coordinates, but as
I understand it that range is only enforced during IK, which I'm sure is a
surprise to users. Now we could by default enforce those as momentum-conserving,
zero coefficient of restitution rigid joint limits. Should we?

@chrisdembia:
I don't think we should do something backwards-incompatible to something so
important. I think we could have an additional flag like
"turn_range_into_motion_constraints" or something named much better.

I definitely think we should exploit that range property though to avoid
redundancy and confusion.
*/

#include <OpenSim/OpenSim.h>

#define USE_RIGID_CONTACT

using namespace OpenSim;
using namespace SimTK;

int main()
{
    // Physical parameters.
    const double mass        = 1.;
    const double radius      = 0.25;
    const double stiffness   = 1.e7;
    const double dissipation = 0.1;
    const double mu_static   = 0.6;
    const double mu_dynamic  = 0.4;
    const double mu_viscous  = 0.01;
    const double restitution = 0.5;

    // Simulation parameters.
    const double initialHeight   = 10.*radius;
    const double initialHorizVel = 1.;
    const double simDuration     = 0.15;
    const double simStepSize     = 1.e-2;

    // Create model with gravity.
    auto osimModel = Model();
    osimModel.setName("BallDrop");
    osimModel.setGravity(Vec3(0, -9.81, 0));

    // Create unconstrained ball.
    auto ball = new OpenSim::Body("ball", mass, Vec3(0),
                                  mass*SimTK::Inertia::sphere(0.1));
    ball->scale(Vec3(radius), false);
    auto freeJoint = new FreeJoint("freeJoint", osimModel.getGround(), *ball);
    osimModel.addBody(ball);
    osimModel.addJoint(freeJoint);

    // Define contact geometry.
    auto ground = osimModel.updGround();
    auto groundContact = new ContactHalfSpace(Vec3(0), Vec3(0, 0, -0.5*SimTK_PI),
                                              ground, "groundContact");
    osimModel.addContactGeometry(groundContact);
    auto ballContact = new ContactSphere(radius, Vec3(0), *ball, "ballContact");
    osimModel.addContactGeometry(ballContact);

    // Define contact forces.
#ifdef USE_RIGID_CONTACT
    auto contactParams = new OpenSim::RigidContactForce::ContactParameters(
        restitution, mu_static, mu_dynamic, mu_viscous);
    contactParams->addGeometry("groundContact");
    contactParams->addGeometry("ballContact");
    auto contact = new OpenSim::RigidContactForce(contactParams);
#else
    auto contactParams = new OpenSim::HuntCrossleyForce::ContactParameters(
        stiffness, dissipation, mu_static, mu_dynamic, mu_viscous);
    contactParams->addGeometry("groundContact");
    contactParams->addGeometry("ballContact");
    auto contact = new OpenSim::HuntCrossleyForce(contactParams);
#endif
    osimModel.addForce(contact);

    // Initialize the system.
    auto& state = osimModel.initSystem();
    freeJoint->getCoordinate(FreeJoint::Coord::TranslationY)
        .setValue(state, initialHeight);
    freeJoint->getCoordinate(FreeJoint::Coord::TranslationX)
        .setSpeedValue(state, initialHorizVel);

    // Create integrator, manager, and force reporter.
    SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
    integrator.setAccuracy(1.e-4);
    Manager manager(osimModel, integrator);
    manager.setInitialTime(0.);

    // Simulate.
    std::cout << "Integrating...";
    double currTime = 0.;
    while (currTime < simDuration) {
        currTime += simStepSize;
        manager.setFinalTime(currTime);
        manager.integrate(state);
    }
    std::cout << " done" << std::endl;

    // TODO: Check the results.

    return 0;
}
