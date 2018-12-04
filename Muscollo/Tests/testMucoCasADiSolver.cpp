#include <Muscollo/osimMuscollo.h>

#include <Muscollo/MucoCasADiSolver/MucoCasADiSolver.h>

using namespace tropter;

using namespace OpenSim;

std::unique_ptr<Model> createSlidingMassModel() {
    auto model = make_unique<Model>();
    model->setName("sliding_mass");
    model->set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", 10.0, SimTK::Vec3(0), SimTK::Inertia(0));
    model->addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("slider", model->getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model->addComponent(joint);

    auto* actu = new CoordinateActuator();
    actu->setCoordinate(&coord);
    actu->setName("actuator");
    actu->setOptimalForce(1);
    actu->setMinControl(-10);
    actu->setMaxControl(10);
    model->addComponent(actu);

    body->attachGeometry(new Sphere(0.05));

    return model;
}

int main() {
    MucoTool muco;
    auto& problem = muco.updProblem();
    problem.setModel(createSlidingMassModel());
    problem.setTimeBounds(MucoInitialBounds(0), MucoFinalBounds(0, 10));
    problem.setStateInfo("/slider/position/value", MucoBounds(0, 1),
            MucoInitialBounds(0), MucoFinalBounds(1));
    problem.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
    problem.addCost<MucoFinalTimeCost>();

    // TODO: Fix the MucoTool interface for MucoCasADiSolver.
    muco.initCasADiSolver();
    MucoSolution solution = muco.solve();

    muco.visualize(solution);
    return 0;
}
