/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: sandboxMuscle.cpp                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

// Some of this code is based on testSingleMuscle,
// testSingleMuscleDeGrooteFregly2016.


#include <Muscollo/osimMuscollo.h>

#include <MuscolloSandboxShared.h>
#include <DeGrooteFregly2016Muscle.h>

#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>

using namespace OpenSim;

// TODO move into the actual test case.
void testDeGrooteFregly2016Muscle() {

    Model model;
    DeGrooteFregly2016Muscle muscle;
    muscle.addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    muscle.addNewPathPoint("insertion", model.updGround(),
            SimTK::Vec3(1.0, 0, 0));
    muscle.finalizeFromProperties();
    muscle.printCurvesToSTOFiles();

    printMessage("%f %f %f %f %f %f\n",
            muscle.calcTendonForceMultiplier(1),
            muscle.calcPassiveForceMultiplier(1),
            muscle.calcActiveForceLengthMultiplier(1),
            muscle.calcForceVelocityMultiplier(-1),
            muscle.calcForceVelocityMultiplier(0),
            muscle.calcForceVelocityMultiplier(1));


    // Test that the force-velocity curve inverse is correct.
    // ------------------------------------------------------
    const auto normFiberVelocity = createVectorLinspace(100, -1, 1);
    for (int i = 0; i < normFiberVelocity.nrow(); ++i) {
        const SimTK::Real& vMTilde = normFiberVelocity[i];
        SimTK_TEST_EQ(muscle.calcForceVelocityInverseCurve(
                muscle.calcForceVelocityMultiplier(vMTilde)), vMTilde);
    }

    // solveBisection().
    // -----------------
    {
        auto calcResidual = [](const SimTK::Real& x) {
            return x - 3.78;
        };
        {
            const auto root =
                    muscle.solveBisection(calcResidual, -5, 5, 1e-6, 1e-12);
            SimTK_TEST_EQ_TOL(root, 3.78, 1e-6);
            // Make sure the x tolerance has an effect.
            SimTK_TEST_NOTEQ_TOL(root, 3.78, 1e-10);
        }
        {
            const auto root =
                    muscle.solveBisection(calcResidual, -5, 5, 1e-10, 1e-12);
            SimTK_TEST_EQ_TOL(root, 3.78, 1e-10);
        }
        // Make sure the y tolerance has an effect.
        {
            const auto root =
                    muscle.solveBisection(calcResidual, -5, 5, 1e-12, 1e-4);
            const auto residual = calcResidual(root);
            SimTK_TEST_EQ_TOL(residual, 0, 1e-4);
            // Make sure the x tolerance has an effect.
            SimTK_TEST_NOTEQ_TOL(residual, 0, 1e-10);
        }
        {
            const auto root =
                    muscle.solveBisection(calcResidual, -5, 5, 1e-12, 1e-10);
            const auto residual = calcResidual(root);
            SimTK_TEST_EQ_TOL(residual, 0, 1e-10);
        }
    }
    {
        auto parabola = [](const SimTK::Real& x) {
            return SimTK::square(x - 2.5);
        };
        SimTK_TEST_MUST_THROW_EXC(muscle.solveBisection(parabola, -5, 5),
                Exception);
    }


}

Model createHangingMuscleModel() {
    Model model;
    model.setName("isometric_muscle");
    model.set_gravity(SimTK::Vec3(9.81, 0, 0));
    auto* body = new Body("body", 0.5, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("joint", model.getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("height");
    model.addComponent(joint);

    /*
    */
    auto* actu = new DeGrooteFregly2016Muscle();
    actu->setName("actuator");
    actu->set_max_isometric_force(30.0);
    actu->set_optimal_fiber_length(0.10);
    actu->set_tendon_slack_length(0.05);
    actu->set_tendon_strain_at_one_norm_force(0.10);

    //actu->set_max_contraction_velocity(10);
    actu->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    actu->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addForce(actu);


    /*
    auto* actu = new Millard2012EquilibriumMuscle();
    // TODO actu->set_fiber_damping(0);
    actu->setName("actuator");
    actu->set_max_isometric_force(30.0);
    actu->set_optimal_fiber_length(0.10);
    actu->set_ignore_tendon_compliance(false);
    actu->set_tendon_slack_length(0.05);
    actu->set_pennation_angle_at_optimal(0); // TODO 0.1);
    actu->set_max_contraction_velocity(10);
    actu->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    actu->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addForce(actu);
    */

    /*
    auto* actu = new ActivationCoordinateActuator();
    actu->setName("actuator");
    actu->setCoordinate(&coord);
    actu->set_activation_time_constant(0.001);
    actu->set_optimal_force(30);
    model.addComponent(actu);

    auto* contr = new PrescribedController();
    contr->setName("controller");
    contr->addActuator(*actu);
    contr->prescribeControlForActuator("actuator", new Constant(1.0));
    model.addComponent(contr);
    */

    body->attachGeometry(new Sphere(0.05));

    return model;
}

int main() {
    testDeGrooteFregly2016Muscle();


    MucoTool muco;
    MucoProblem& mp = muco.updProblem();
    Model model = createHangingMuscleModel();

    auto* controller = new PrescribedController();
    controller->addActuator(model.getComponent<Actuator>("actuator"));
    controller->prescribeControlForActuator("actuator", new Constant(0.10));
    model.addController(controller);
    model.finalizeFromProperties();

    TableReporter* rep = new TableReporter();
    /*
    rep->addToReport(model.getComponent("actuator/geometrypath").getOutput("length"));
    rep->addToReport(model.getComponent("actuator").getOutput("normalized_fiber_length"));
    rep->addToReport(model.getComponent("actuator").getOutput("normalized_fiber_velocity"));
    // rep->addToReport(model.getComponent("actuator").getOutput("fiber_velocity"));
    rep->addToReport(model.getComponent("actuator").getOutput("force_velocity_multiplier"));
    rep->set_report_time_interval(0.001);
    model.addComponent(rep);
     */

    SimTK::State state = model.initSystem();
    const auto& actuator = model.getComponent("actuator");
    const auto* dgf = dynamic_cast<const DeGrooteFregly2016Muscle*>(&actuator);
    bool usingDGF = dgf != nullptr;
    bool hasFiberDynamics = !(
            usingDGF ? dgf->get_ignore_tendon_compliance() :
            dynamic_cast<const Muscle&>(actuator).get_ignore_tendon_compliance());


    if (hasFiberDynamics) {
        model.setStateVariableValue(state, "actuator/activation", 0.01);
        model.setStateVariableValue(state, "joint/height/value", 0.15);
        if (usingDGF) {
            model.realizePosition(state);
            model.getComponent<DeGrooteFregly2016Muscle>("actuator").computeInitialFiberEquilibrium(state);
            std::cout << "DEBUG " << model.getStateVariableValue(state, "actuator/norm_fiber_length")
                    << std::endl;
        } else {
            model.equilibrateMuscles(state);
        }
    }

    // TODO
    // Manager manager(model, state);
    // manager.getIntegrator().setAccuracy(1e-10);
    // manager.getIntegrator().setMaximumStepSize(0.001);
    // manager.integrate(2.0);
    // STOFileAdapter::write(rep->getTable(), "DEBUG_sandboxMuscle_reporter.sto");
    // visualize(model, manager.getStateStorage());
    // std::exit(-1);

    //std::cout << "DEBUG " <<
    //        model.getStateVariableValue(state, "actuator/fiber_length")
    //        << std::endl;
    //model.equilibrateMuscles(state);
    //std::cout << "DEBUG " <<
    //        model.getStateVariableValue(state, "actuator/fiber_length")
    //        << std::endl;
    mp.setModel(model);
    mp.setTimeBounds(0, {0.05, 1.0});
    // TODO this might have been the culprit when using the Millard muscle:
    // TODO TODO TODO
    mp.setStateInfo("joint/height/value", {0.10, 0.20}, 0.15, 0.14);
    mp.setStateInfo("joint/height/speed", {-10, 10}, 0, 0);
    // TODO initial fiber length?
    // TODO how to enforce initial equilibrium?
    if (hasFiberDynamics) {
        if (usingDGF) {
            std::cout << "DEBUG norm_fiber_length" <<
                    model.getStateVariableValue(state, "actuator/norm_fiber_length") << std::endl;
            // We would prefer to use a range of [0.2, 1.8] but then IPOPT
            // tries very small fiber lengths that cause tendon stretch to be
            // HUGE, causing insanely high tendon forces.
            mp.setStateInfo("actuator/norm_fiber_length", {0.8, 1.8},
                    model.getStateVariableValue(state, "actuator/norm_fiber_length"));
        } else {
            mp.setStateInfo("actuator/fiber_length", {0, 0.3},
                    model.getStateVariableValue(state, "actuator/fiber_length"));
        }
    }
    // OpenSim might not allow activations of 0.
    mp.setStateInfo("actuator/activation", {0.01, 1}, 0.01);
    mp.setControlInfo("actuator", {0.01, 1});

    mp.addCost(MucoFinalTimeCost());

    // TODO try ActivationCoordinateActuator first.
    // TODO i feel like the force-velocity effect is much more strict than it
    // should be.

    MucoTropterSolver& solver = muco.initSolver();
    // solver.set_num_mesh_points(20);
    // TODO set initial guess from forward simulation.
    // TODO for this to work, we want to enable solving for equilibrium fiber
    // length.
    solver.setGuess("forward-simulation");
    MucoIterate guessForwardSim = solver.createGuess("forward-simulation");
    guessForwardSim.write("DEBUG_forward_sim.sto");
    std::cout << "DEBUGguessForwardSim " << guessForwardSim.getStatesTrajectory() << std::endl;
    muco.visualize(guessForwardSim);

    solver.setGuess(guessForwardSim);

    MucoSolution solution = muco.solve().unseal();
    solution.write("sandboxMuscle_solution.sto");
    std::cout << "DEBUG joint/height/value "
            << solution.getState("joint/height/value") << std::endl;
    std::cout << "DEBUG joint/height/speed "
            << solution.getState("joint/height/speed") << std::endl;

    std::exit(-1);

    // TODO perform forward simulation using optimized controls; see if we
    // end up at the correct final state.
    {

        // Add a controller to the model.
        const SimTK::Vector& time = solution.getTime();
        const auto control = solution.getControl("actuator");
        auto* controlFunction = new GCVSpline(5, time.nrow(), &time[0],
                &control[0]);
        auto* controller = new PrescribedController();
        controller->addActuator(model.getComponent<Actuator>("actuator"));
        controller->prescribeControlForActuator("actuator", controlFunction);
        model.addController(controller);

        // Set the initial state.
        SimTK::State state = model.initSystem();
        model.setStateVariableValue(state, "joint/height/value", 0.15);
        model.setStateVariableValue(state, "actuator/activation", 0);

        // Integrate.
        Manager manager(model, state);
        SimTK::State finalState = manager.integrate(time[time.nrow() - 1]);
        std::cout << "DEBUG forward sim joint/height/value history "
                << model.getStateVariableValue(finalState, "joint/height/value")
                << std::endl;
        SimTK_TEST_EQ_TOL(
                model.getStateVariableValue(finalState, "joint/height/value"),
                0.14, 1e-4);
        manager.getStateStorage().print("sandboxMuscle_timestepping.sto");
    }


    // TODO support constraining initial fiber lengths to their equilibrium
    // lengths in Tropter!!!!!!!!!!!!!! (in explicit mode).

    return EXIT_SUCCESS;
}

