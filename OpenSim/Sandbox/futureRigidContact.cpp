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
