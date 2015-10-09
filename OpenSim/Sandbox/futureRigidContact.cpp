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
    Model *osimModel = new Model;
    osimModel->setName("BallDrop");
    osimModel->setGravity(Vec3(0, -9.81, 0));

    // Create unconstrained ball.
    OpenSim::Body ball("ball", mass, Vec3(0), mass*SimTK::Inertia::sphere(0.1));
    ball.scale(Vec3(radius), false);
    FreeJoint free("", osimModel->getGround().getName(), "ball");
    osimModel->addBody(&ball);
    osimModel->addJoint(&free);

    // Define contact geometry.
    OpenSim::Ground& ground = osimModel->updGround();
    ContactHalfSpace groundContact(Vec3(0), Vec3(0, 0, -0.5*SimTK_PI), ground,
                                   "groundContact");
    osimModel->addContactGeometry(&groundContact);
    ContactSphere ballContact(radius, Vec3(0), ball, "ballContact");
    osimModel->addContactGeometry(&ballContact);

    // Define contact forces.
#ifdef USE_RIGID_CONTACT
    OpenSim::RigidContactForce::ContactParameters
        contactParams(restitution, mu_static, mu_dynamic, mu_viscous);
    contactParams.addGeometry("groundContact");
    contactParams.addGeometry("ballContact");
    OpenSim::RigidContactForce contact(&contactParams);
#else
    OpenSim::HuntCrossleyForce::ContactParameters
        contactParams(stiffness, dissipation, mu_static, mu_dynamic, mu_viscous);
    contactParams.addGeometry("groundContact");
    contactParams.addGeometry("ballContact");
    OpenSim::HuntCrossleyForce contact(&contactParams);
#endif
    osimModel->addForce(&contact);

    // Initialize the system.
    SimTK::State& state = osimModel->initSystem();
    CoordinateSet& modelCoordSet = osimModel->updCoordinateSet();
    modelCoordSet[0].setValue(state, initialHorizVel); //velocity along X-axis
    modelCoordSet[4].setValue(state, initialHeight); //displacement along Y-axis

    // Create integrator, manager, and force reporter.
    SimTK::RungeKuttaMersonIntegrator integrator(osimModel->getMultibodySystem());
    integrator.setAccuracy(1.e-4);
    Manager manager(*osimModel, integrator);
    manager.setInitialTime(0.);

    // Simulate.
    double currTime = 0.;
    while (currTime < simDuration) {
        currTime += simStepSize;
        manager.setFinalTime(currTime);
        manager.integrate(state);
    }

    // TODO: Check the results.

    osimModel->disownAllComponents();

    return 0;
}
