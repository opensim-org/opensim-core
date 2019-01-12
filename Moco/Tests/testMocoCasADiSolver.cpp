#include <Moco/osimMoco.h>

#include <Moco/MocoCasADiSolver/MocoCasADiSolver.h>

using namespace tropter;

using namespace OpenSim;

// TODO: Could remove this test, as it duplicates testMocoInterface.

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
    MocoTool moco;
    auto& problem = moco.updProblem();
    problem.setModel(createSlidingMassModel());
    problem.setTimeBounds(MocoInitialBounds(0), MocoFinalBounds(0, 10));
    problem.setStateInfo("/slider/position/value", MocoBounds(0, 1),
            MocoInitialBounds(0), MocoFinalBounds(1));
    problem.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
    problem.addCost<MocoFinalTimeCost>();

    auto& solver = moco.initCasADiSolver();
    solver.set_num_mesh_points(20);
    MocoSolution solution = moco.solve();

    // moco.visualize(solution);
    return 0;
}
