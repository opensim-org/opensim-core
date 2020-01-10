/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxContact.cpp                                           *
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

#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/Model/PointToPointSpring.h>
//#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Analyses/ForceReporter.h>
#include <Moco/osimMoco.h>
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Simulation/Control/PrescribedController.h>

#include <MocoSandboxShared.h>

// TODO achieve sliding friction (apply constant tangential force,
// maybe from gravity?).

// TODO add tests for contact model (energy conservation) to make sure, for
// example, ball bounces back to original height if there is no dissipation.

using namespace OpenSim;
using SimTK::Vec3;

class CustomContactForce : public Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(CustomContactForce, Force);
public:
    OpenSim_DECLARE_SOCKET(station, Station, "TODO");
    void computeForce(const SimTK::State& s,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& /*generalizedForces*/) const override {
        const auto& pt = getConnectee<Station>("station");
        const auto& pos = pt.getLocationInGround(s);
        const auto& vel = pt.getVelocityInGround(s);
        const SimTK::Real y = pos[1];
        const SimTK::Real vy = vel[1];
        SimTK::Vec3 force(0);
        const SimTK::Real depth = 0 - y;
        const SimTK::Real depthRate = 0 - vy;
        const SimTK::Real a = 5e7; // N/m^3
        const SimTK::Real b = 1.0; // s/m
        if (depth > 0) {
            //force = 1000 * depth;
            force[1] = fmax(0, a * pow(depth, 3) * (1 + b * depthRate));
        }
        const SimTK::Real voidStiffness = 1.0; // N/m
        force[1] += voidStiffness * depth;
        //applyGeneralizedForce(s, getModel().getCoordinateSet().get(0),
        //        force, generalizedForces);
        applyForceToPoint(s, pt.getParentFrame(), pt.get_location(), force,
                bodyForces);
        // TODO equal and opposite force on ground.
    }
};

Model createModel() {
    Model model;
    model.setName("point_mass");
    auto* body = new Body("body", 50.0, Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("slider",
            model.getGround(), Vec3(0), SimTK::Vec3(0, 0, 0.5 * SimTK::Pi),
            *body, Vec3(0), Vec3(0));
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("y");
    model.addComponent(joint);

    auto* station = new Station();
    station->setName("contact_point");
    station->connectSocket_parent_frame(*body);
    model.addComponent(station);

    auto* force = new CustomContactForce();
    force->setName("contact");
    model.addComponent(force);
    force->connectSocket_station(*station);

    return model;
}

std::unique_ptr<Model> createModel2D() {
    auto model = make_unique<Model>();
    model->setName("point_mass");
    auto* intermed = new Body("intermed", 0, Vec3(0), SimTK::Inertia(0));
    model->addComponent(intermed);
    // TODO inertia...
    auto* body = new Body("body", 50.0, Vec3(0), SimTK::Inertia(1));
    model->addComponent(body);

    // Allows translation along x.
    auto* jointX = new SliderJoint();
    jointX->setName("tx");
    jointX->connectSocket_parent_frame(model->getGround());
    jointX->connectSocket_child_frame(*intermed);
    auto& coordX = jointX->updCoordinate(SliderJoint::Coord::TranslationX);
    coordX.setName("tx");
    model->addComponent(jointX);

    // The joint's x axis must point in the global "+y" direction.
    auto* jointY = new SliderJoint("ty",
            *intermed, Vec3(0), Vec3(0, 0, 0.5 * SimTK::Pi),
            *body, Vec3(0), Vec3(0, 0, 0.5 * SimTK::Pi));
    auto& coordY = jointY->updCoordinate(SliderJoint::Coord::TranslationX);
    coordY.setName("ty");
    model->addComponent(jointY);

    auto* station = new Station();
    station->setName("contact_point");
    station->connectSocket_parent_frame(*body);
    model->addComponent(station);

    //auto* force = new CustomContactForce();
    auto* force = new AckermannVanDenBogert2010Force();
    force->setName("contact");
    model->addComponent(force);
    force->connectSocket_station(*station);

    return model;
}

void ball2d() {

    const SimTK::Real y0 = 0.5;
    const SimTK::Real vx0 = 0.7;
    const SimTK::Real finalTime = 1.0;

    auto model = createModel2D();
    auto state = model->initSystem();
    model->setStateVariableValue(state, "ty/ty/value", y0);
    model->setStateVariableValue(state, "tx/tx/speed", vx0);
    Manager manager(*model);
    // Without the next line: Simbody takes steps that are too large and NaNs
    // are generated.
    manager.setIntegratorMaximumStepSize(0.05);
    manager.initialize(state);
    state = manager.integrate(finalTime);
    const auto& statesTimeStepping = manager.getStateStorage();
    std::cout << "DEBUG " << statesTimeStepping.getSize() << std::endl;
    visualize(*model, statesTimeStepping);
    auto statesTimeSteppingTable = statesTimeStepping.exportToTable();
    STOFileAdapter::write(statesTimeSteppingTable, "ball2d_timestepping.sto");

    MocoStudy study;
    study.setName("ball2d");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& mp = study.updProblem();

    // Model (dynamics).
    // -----------------
    mp.setModel(std::move(model));

    // Bounds.
    // -------
    // Initial time must be 0, final time can be within [0, 5].
    mp.setTimeBounds(0, finalTime);

    mp.setStateInfo("/tx/tx/value", {-5, 5}, 0);
    mp.setStateInfo("/ty/ty/value", {-0.5, 1}, y0);
    mp.setStateInfo("/tx/tx/speed", {-10, 10}, vx0);
    mp.setStateInfo("/ty/ty/speed", {-10, 10}, 0);

    // Configure the solver.

    MocoTropterSolver& ms = study.initSolver();
    ms.set_num_mesh_intervals(500);

    MocoTrajectory guess = ms.createGuess();

    // Setting this guess reduces the number of iterations from 90 to 6.
    // Can tweak the guess to test convergence (~50 iterations):
    //statesTimeSteppingTable.updDependentColumn("ty/ty/value") += 0.05;
    guess.setStatesTrajectory(statesTimeSteppingTable);

    ms.setGuess(guess);

    // TODO interface for setting these options:
    // TODO ms.setOption("optim.hessian-approximation", "limited-memory");
    // TODO ms.set_optimizer_algorithm("ipopt");


    // Now that we've finished setting up the tool, print it to a file.
    //moco.print("contact.omoco");

    // Solve the problem.
    // ==================
    MocoSolution solution = study.solve();

    solution.write("ball2d_solution.sto");

    std::cout << "RMS: " << solution.compareContinuousVariablesRMS(guess)
            << std::endl;

    // Visualize.
    // ==========
    study.visualize(solution);
}

std::unique_ptr<Model> createModelPendulum(double linkLength, double jointHeight,
        double dissipation, double frictionCoeff) {
    auto model = make_unique<Model>();
    model->setName("pendulum");
    auto* body = new Body("body", 50.0, Vec3(0), SimTK::Inertia(1));
    model->addComponent(body);

    // The joint's x axis must point in the global "+y" direction.
    auto* joint = new PinJoint("rz",
            model->getGround(), Vec3(0, jointHeight, 0), Vec3(0),
            *body, Vec3(-linkLength, 0, 0), Vec3(0));
    auto& rz = joint->updCoordinate(PinJoint::Coord::RotationZ);
    rz.setName("rz");
    model->addComponent(joint);

    auto* station = new Station();
    station->setName("contact_point");
    station->connectSocket_parent_frame(*body);
    model->addComponent(station);

    auto* force = new AckermannVanDenBogert2010Force();
    force->set_dissipation(dissipation);
    force->set_friction_coefficient(frictionCoeff);
    force->setName("contact");
    model->addComponent(force);
    force->connectSocket_station(*station);

    return model;
}

void pendulum() {
    const double jointHeight = 0.6;
    const double linkLength = 1.0;
    const double dissipation = 1.0;
    const double frictionCoeff = 1.0;
    const SimTK::Real finalTime = 1.0;
    auto model = createModelPendulum(linkLength, jointHeight, dissipation,
            frictionCoeff);
    auto state = model->initSystem();
    Manager manager(*model);
    // Without the next line: Simbody takes steps that are too large and NaNs
    // are generated.
    manager.setIntegratorMaximumStepSize(0.05);
    manager.initialize(state);
    state = manager.integrate(finalTime);
    const auto& statesTimeStepping = manager.getStateStorage();
    visualize(*model, statesTimeStepping);
    auto statesTimeSteppingTable = statesTimeStepping.exportToTable();
    STOFileAdapter::write(statesTimeSteppingTable, "pendulum_timestepping.sto");

    MocoStudy study;
    study.setName("ball2d");
    MocoProblem& mp = study.updProblem();
    mp.setModel(std::move(model));
    mp.setTimeBounds(0, finalTime);

    mp.setStateInfo("/rz/rz/value", {-0.5 * SimTK::Pi, 0.5 * SimTK::Pi}, 0);
    mp.setStateInfo("/rz/rz/speed", {-10, 10}, 0);

    // Configure the solver.

    MocoTropterSolver& ms = study.initSolver();
    ms.set_num_mesh_intervals(500);
    MocoTrajectory guess = ms.createGuess();
    guess.setStatesTrajectory(statesTimeSteppingTable);

    ms.setGuess(guess);
    MocoSolution solution = study.solve();

    solution.write("pendulum_solution.sto");

    std::cout << "RMS: " << solution.compareContinuousVariablesRMS(guess)
            << std::endl;

    // Visualize.
    // ==========
    study.visualize(solution);
}

std::unique_ptr<Model> createModelPendulumActivationCoordinateActuator() {
    const double jointHeight = 0.6;
    const double linkLength = 1.0;
    auto model = make_unique<Model>();
    model->setName("pendulum");
    auto* body = new Body("body", 50.0, Vec3(0), SimTK::Inertia(1));
    model->addComponent(body);

    // The joint's x axis must point in the global "+y" direction.
    auto* joint = new PinJoint("rz",
            model->getGround(), Vec3(0, jointHeight, 0), Vec3(0),
            *body, Vec3(-linkLength, 0, 0), Vec3(0));
    auto& rz = joint->updCoordinate(PinJoint::Coord::RotationZ);
    rz.setName("rz");
    model->addComponent(joint);

    auto* station = new Station();
    station->setName("contact_point");
    station->connectSocket_parent_frame(*body);
    model->addComponent(station);

    auto* force = new AckermannVanDenBogert2010Force();
    force->set_dissipation(1.0);
    force->set_friction_coefficient(1.0);
    force->setName("contact");
    model->addComponent(force);
    force->connectSocket_station(*station);

    auto* actu = new ActivationCoordinateActuator();
    actu->set_default_activation(0.1);
    actu->setName("actuator");
    actu->setCoordinate(&rz);
    actu->setOptimalForce(600);
    model->addComponent(actu);

    return model;
}

void pendulumActivationCoordinateActuator() {
    const SimTK::Real finalTime = 1.0;
    auto model = createModelPendulumActivationCoordinateActuator();
    PrescribedController* contr = new PrescribedController();
    contr->addActuator(model->getComponent<ActivationCoordinateActuator>
            ("actuator"));
    contr->prescribeControlForActuator("actuator", new Constant(1.0));
    model->addComponent(contr);
    auto state = model->initSystem();
    Manager manager(*model);
    // Without the next line: Simbody takes steps that are too large and NaNs
    // are generated.
    manager.setIntegratorMaximumStepSize(0.05);
    manager.initialize(state);
    state = manager.integrate(finalTime);
    const auto& statesTimeStepping = manager.getStateStorage();
    visualize(*model, statesTimeStepping);
    auto statesTimeSteppingTable = statesTimeStepping.exportToTable();
    STOFileAdapter::write(statesTimeSteppingTable,
            "pendulumaca_timestepping.sto");

    /*

    MocoStudy study;
    study.setName("pendulumaca");
    MocoProblem& mp = study.updProblem();
    mp.setModel(model);
    mp.setTimeBounds(0, finalTime);

    mp.setStateInfo("/rz/rz/value", {-0.5 * SimTK::Pi, 0.5 * SimTK::Pi}, 0);
    mp.setStateInfo("/rz/rz/speed", {-10, 10}, 0);

    // Configure the solver.

    MocoTropterSolver& ms = study.initSolver();
    ms.set_num_mesh_intervals(500);
    MocoTrajectory guess = ms.createGuess();
    guess.setStatesTrajectory(statesTimeSteppingTable);

    ms.setGuess(guess);
    MocoSolution solution = study.solve();

    solution.write("pendulumaca_solution.sto");

    std::cout << "RMS: " << solution.compareStatesControlsRMS(guess)
     << std::endl;

    // Visualize.
    // ==========
    study.visualize(solution);
     */
}

void addResidualActivationCoordinateActuators(Model& model) {

    auto& coordSet = model.updCoordinateSet();
    
    auto* res_rz = new ActivationCoordinateActuator();
    res_rz->setCoordinate(&coordSet.get("rz"));
    res_rz->setName("res_rz");
    res_rz->setOptimalForce(100);
    model.addComponent(res_rz);

    auto* res_tx = new ActivationCoordinateActuator();
    res_tx->setCoordinate(&coordSet.get("tx"));
    res_tx->setName("res_tx");
    res_tx->setOptimalForce(2500);
    model.addComponent(res_tx);

    auto* res_ty = new ActivationCoordinateActuator();
    res_ty->setCoordinate(&coordSet.get("ty"));
    res_ty->setName("res_ty");
    res_ty->setOptimalForce(2500);
    model.addComponent(res_ty);

}

void addResidualCoordinateActuators(Model& model) {

    auto& coordSet = model.updCoordinateSet();

    auto* res_rz = new CoordinateActuator();
    res_rz->setCoordinate(&coordSet.get("rz"));
    res_rz->setName("res_rz");
    res_rz->setOptimalForce(100);
    model.addComponent(res_rz);

    auto* res_tx = new CoordinateActuator();
    res_tx->setCoordinate(&coordSet.get("tx"));
    res_tx->setName("res_tx");
    res_tx->setOptimalForce(2500);
    model.addComponent(res_tx);

    auto* res_ty = new CoordinateActuator();
    res_ty->setCoordinate(&coordSet.get("ty"));
    res_ty->setName("res_ty");
    res_ty->setOptimalForce(2500);
    model.addComponent(res_ty);

}

std::unique_ptr<Model> createModelSLIP() {
    auto model = make_unique<Model>();
    model->setName("SLIP");
    auto* foot = new Body("foot", 15.0, Vec3(0), SimTK::Inertia(1));
    model->addComponent(foot);
    auto* pelvis = new Body("pelvis", 35.0, Vec3(0), SimTK::Inertia(1));
    model->addComponent(pelvis);

    Sphere bodyGeom(0.05);
    bodyGeom.setColor(SimTK::Red);
    foot->attachGeometry(bodyGeom.clone());
    bodyGeom.setColor(SimTK::Blue);
    pelvis->attachGeometry(bodyGeom.clone());

    auto* planar = new PlanarJoint("planar", model->getGround(), *foot);
    planar->updCoordinate(PlanarJoint::Coord::TranslationX).setName("tx");
    auto& ty = planar->updCoordinate(PlanarJoint::Coord::TranslationY);
    ty.setName("ty");
    ty.setDefaultValue(0.1);
    planar->updCoordinate(PlanarJoint::Coord::RotationZ).setName("rz");
    model->addComponent(planar);

    const Vec3 rz90 = SimTK::Vec3(0, 0, 0.5 * SimTK::Pi);
    auto* leg = new SliderJoint("leg", *foot, Vec3(0), rz90,
            *pelvis, Vec3(0), rz90);
    auto& length = leg->updCoordinate(SliderJoint::Coord::TranslationX);
    length.setName("length");
    length.setDefaultValue(1.0);
    model->addComponent(leg);

    // Foot-ground contact.
    auto* station = new Station();
    station->setName("contact_point");
    station->connectSocket_parent_frame(*foot);
    model->addComponent(station);

    auto* force = new AckermannVanDenBogert2010Force();
    force->set_dissipation(1.0);
    force->set_stiffness(5e5);
    force->set_friction_coefficient(1.0);
    force->setName("contact");
    model->addComponent(force);
    force->connectSocket_station(*station);

    // auto* force = new MeyerFregly2016Force();
    // // force->set_dissipation(1.0);
    // // force->set_stiffness(1e4);
    // force->setName("contact");
    // model->addComponent(force);
    // force->connectSocket_station(*station);

    // auto* force = new EspositoMiller2018Force();
    // force->set_stiffness(1e5);
    // force->setName("contact");
    // model->addComponent(force);
    // force->connectSocket_station(*station);


    // Leg muscles.
    // TODO spring force should be felt all the time; similar to contact.
    // TODO disallow PathSpring?
    auto* spring = new PointToPointSpring();
    model->addComponent(spring);
    spring->set_stiffness(5e3);
    spring->set_rest_length(1.0);
    spring->connectSocket_body1(*foot);
    spring->connectSocket_body2(*pelvis);

    auto* foot_marker = new Marker("foot_COM", *foot, Vec3(0));
    model->addComponent(foot_marker);
    auto* pelvis_marker = new Marker("pelvis_COM", *pelvis, Vec3(0));
    model->addComponent(pelvis_marker);

    return model;
}

std::unique_ptr<Model>createModelSLIPActuated() {
    auto model = make_unique<Model>();
    model->setName("SLIP");
    auto* foot = new Body("foot", 15.0, Vec3(0), SimTK::Inertia(1));
    model->addComponent(foot);
    auto* pelvis = new Body("pelvis", 35.0, Vec3(0), SimTK::Inertia(1));
    model->addComponent(pelvis);

    Sphere bodyGeom(0.05);
    bodyGeom.setColor(SimTK::Red);
    foot->attachGeometry(bodyGeom.clone());
    bodyGeom.setColor(SimTK::Blue);
    pelvis->attachGeometry(bodyGeom.clone());

    auto* planar = new PlanarJoint("planar", model->getGround(), *foot);
    planar->updCoordinate(PlanarJoint::Coord::TranslationX).setName("tx");
    auto& ty = planar->updCoordinate(PlanarJoint::Coord::TranslationY);
    ty.setName("ty");
    ty.setDefaultValue(0.1);
    planar->updCoordinate(PlanarJoint::Coord::RotationZ).setName("rz");
    model->addComponent(planar);

    const Vec3 rz90 = SimTK::Vec3(0, 0, 0.5 * SimTK::Pi);
    auto* leg = new SliderJoint("leg", *foot, Vec3(0), rz90,
            *pelvis, Vec3(0), rz90);
    auto& length = leg->updCoordinate(SliderJoint::Coord::TranslationX);
    length.setName("length");
    length.setDefaultValue(1.0);
    model->addComponent(leg);

    // Foot-ground contact.
    auto* station = new Station();
    station->setName("contact_point");
    station->connectSocket_parent_frame(*foot);
    model->addComponent(station);

    auto* force = new AckermannVanDenBogert2010Force();
    force->set_dissipation(1.0);
    force->set_stiffness(5e5);
    force->set_friction_coefficient(1.0);
    force->setName("contact");
    model->addComponent(force);
    force->connectSocket_station(*station);

    // auto* force = new MeyerFregly2016Force();
    // force->set_dissipation(1.0);
    // force->set_stiffness(1e4);
    // force->setName("contact");
    // model->addComponent(force);
    // force->connectSocket_station(*station);

    // auto* force = new EspositoMiller2018Force();
    // force->set_stiffness(1e5);
    // force->setName("contact");
    // model->addComponent(force);
    // force->connectSocket_station(*station);

    auto* actuator = new CoordinateActuator();
    actuator->setCoordinate(&length);
    actuator->setName("actuator");
    actuator->setOptimalForce(2500);
    //actuator->set_activation_time_constant(0.025);
    model->addComponent(actuator);

    //addResidualActivationCoordinateActuators(*model);
    addResidualCoordinateActuators(*model);

    auto* foot_marker = new Marker("foot_COM", *foot, Vec3(0));
    model->addComponent(foot_marker);
    auto* pelvis_marker = new Marker("pelvis_COM", *pelvis, Vec3(0));
    model->addComponent(pelvis_marker);

    return model;
}

void slip(double rzvalue0 = 0, double rzspeed0 = 0) {
    const SimTK::Real finalTime = 1.0;
    auto model = createModelSLIP();
    auto state = model->initSystem();
    model->setStateVariableValue(state, "planar/rz/value", rzvalue0);
    model->setStateVariableValue(state, "planar/rz/speed", rzspeed0);
    Manager manager(*model);
    manager.setIntegratorAccuracy(1e-5);
    manager.setIntegratorMaximumStepSize(0.05);
    manager.initialize(state);
    state = manager.integrate(finalTime);
    const auto& statesTimeStepping = manager.getStateStorage();
    visualize(*model, statesTimeStepping);
    auto statesTimeSteppingTable = statesTimeStepping.exportToTable();
    STOFileAdapter::write(statesTimeSteppingTable, "slip_timestepping.sto");

    MocoStudy study;
    study.setName("slip");
    MocoProblem& mp = study.updProblem();
    mp.setModel(std::move(model));
    mp.setTimeBounds(0, finalTime);
    using SimTK::Pi;
    mp.setStateInfo("/planar/tx/value", {-5, 5}, 0);
    mp.setStateInfo("/planar/ty/value", {-0.5, 2}, 0.1);
    mp.setStateInfo("/planar/rz/value", {-0.5*Pi, 0.5*Pi}, rzvalue0);
    mp.setStateInfo("/leg/length/value", {0.1, 1.9}, 1.0);
    mp.setStateInfo("/planar/tx/speed", {-10, 10}, 0);
    mp.setStateInfo("/planar/ty/speed", {-10, 10}, 0);
    mp.setStateInfo("/planar/rz/speed", {-10, 10}, rzspeed0);
    mp.setStateInfo("/leg/length/speed", {-10, 10}, 0);

    MocoTropterSolver& ms = study.initSolver();
    ms.set_num_mesh_intervals(500);
    //ms.set_optim_max_iterations(2);
    MocoTrajectory guess = ms.createGuess();
    //statesTimeSteppingTable.updMatrix() +=
    //        0.1 * SimTK::Test::randMatrix(guess.getNumTimes(), 6);
    //statesTimeSteppingTable.updDependentColumn("planar/ty/value") +=
    //        0.05 * SimTK::Test::randVector(guess.getNumTimes());
    guess.setStatesTrajectory(statesTimeSteppingTable);
    ms.setGuess(guess);

    MocoSolution solution = study.solve().unseal();
    solution.write("slip_solution.sto");
    std::cout << "RMS: " << solution.compareContinuousVariablesRMS(guess)
            << std::endl;
    study.visualize(solution);
}

void slipSolveForForce(double rzvalue0 = 0, double rzspeed0 = 0) {
    const SimTK::Real finalTime = 0.68;
    auto modelTS = createModelSLIP();
    auto reporter = new TableReporterVec3();
    reporter->setName("contact_rep");
    reporter->set_report_time_interval(0.01);
    reporter->addToReport(
            modelTS->getComponent("contact").getOutput("force_on_station"));
    modelTS->addComponent(reporter);
    auto markersReporter = new TableReporterVec3();
    markersReporter->setName("marker_rep");
    markersReporter->set_report_time_interval(0.01);
    markersReporter->addToReport(
            modelTS->getComponent("foot_COM").getOutput("location"),
            "foot_COM");
    markersReporter->addToReport(
            modelTS->getComponent("pelvis_COM").getOutput("location"),
            "pelvis_COM");
    modelTS->addComponent(markersReporter);
    auto state = modelTS->initSystem();
    modelTS->setStateVariableValue(state, "planar/rz/value", rzvalue0);
    modelTS->setStateVariableValue(state, "planar/rz/speed", rzspeed0);
    auto* forceRep = new ForceReporter(modelTS.get());
    modelTS->addAnalysis(forceRep);
    Manager manager(*modelTS);
    manager.setIntegratorAccuracy(1e-5);
    manager.setIntegratorMaximumStepSize(0.05);
    manager.initialize(state);
    state = manager.integrate(finalTime);
    const auto& statesTimeStepping = manager.getStateStorage();


    // Calculate the signed magnitude of the spring force.
    // forceRep->getForceStorage().print("DEBUG_slipSolveForForce_forces.sto");
    SimTK::Vector springForceActual;
    SimTK::Vector springForceActualTime;
    {
        auto forceRepTable = forceRep->getForcesTable();
        std::string prefix = "pointtopointspring.";
        auto X = forceRepTable.getDependentColumn(prefix + "foot.force.X");
        auto Y = forceRepTable.getDependentColumn(prefix + "foot.force.Y");
        auto Z = forceRepTable.getDependentColumn(prefix + "foot.force.Z");
        auto pX = forceRepTable.getDependentColumn(prefix + "pelvis.point.X") -
                forceRepTable.getDependentColumn(prefix + "foot.point.X");
        auto pY = forceRepTable.getDependentColumn(prefix + "pelvis.point.Y") -
                forceRepTable.getDependentColumn(prefix + "foot.point.Y");
        auto pZ = forceRepTable.getDependentColumn(prefix + "pelvis.point.Z") -
                forceRepTable.getDependentColumn(prefix + "foot.point.Z");
        const auto& stdTime = forceRepTable.getIndependentColumn();
        springForceActualTime =
                SimTK::Vector((int)stdTime.size(), stdTime.data());
        springForceActual.resize(X.size());
        assert(X.size() > 0);
        for (int i = 0; i < X.size(); ++i) {
            SimTK::Vec3 forceVec(X[i], Y[i], Z[i]);
            SimTK::Vec3 lineOfAction(pX[i], pY[i], pZ[i]);
            springForceActual[i] = forceVec.norm();
            if (~lineOfAction * forceVec < 0) {
                springForceActual[i] *= -1;
            }
        }
        forceRepTable.appendColumn("spring_force_mag", springForceActual);
        STOFileAdapter::write(forceRepTable,
                "DEBUG_slipSolveForForce_springforce.sto");
    }

    auto forcesFlattened = reporter->getTable().flatten({"x", "y", "z"});
    //TimeSeriesTable forcesFilt = filterLowpass(forcesFlattened, 18.0, true);
    STOFileAdapter::write(forcesFlattened,
        "slipSolveForForce_contact_force.sto");
    auto statesTimeSteppingTable = statesTimeStepping.exportToTable();
    STOFileAdapter::write(statesTimeSteppingTable,
            "slipSolveForForce_timestepping.sto");
    auto markers = markersReporter->getTable();
    TimeSeriesTable markersFilt = filterLowpass(markers.flatten(), 6.0, true);
    auto markersPacked = markersFilt.pack<double>();
    TimeSeriesTableVec3 markersToUse(markersPacked);
    markersToUse.setColumnLabels({"foot_COM", "pelvis_COM"});
    visualize(*modelTS, statesTimeStepping);

    MocoStudy study;
    study.setName("slipSolveForForce");
    MocoProblem& mp = study.updProblem();
    //mp.setModel(modelTS);
    mp.setModel(createModelSLIPActuated());
    mp.setTimeBounds(0, finalTime);
    using SimTK::Pi;
    mp.setStateInfo("/planar/tx/value", {-5, 5}, 0);
    mp.setStateInfo("/planar/ty/value", {-0.5, 2}, 0.1);
    mp.setStateInfo("/planar/rz/value", {-0.5*Pi, 0.5*Pi}, rzvalue0);
    mp.setStateInfo("/leg/length/value", {0.1, 1.9}, 1.0);
    mp.setStateInfo("/planar/tx/speed", {-10, 10}, 0);
    mp.setStateInfo("/planar/ty/speed", {-10, 10}, 0);
    mp.setStateInfo("/planar/rz/speed", {-10, 10}, rzspeed0);
    mp.setStateInfo("/leg/length/speed", {-10, 10}, 0);
    //mp.setStateInfo("/actuator/activation", {-2, 2});
    //mp.setStateInfo("/res_rz/activation", { -2, 2 });
    //mp.setStateInfo("/res_tx/activation", { -2, 2 });
    //mp.setStateInfo("/res_ty/activation", { -2, 2 });
    mp.setControlInfo("/actuator", {-1, 1});
    mp.setControlInfo("/res_rz", { -1, 1 });
    mp.setControlInfo("/res_tx", { -1, 1 });
    mp.setControlInfo("/res_ty", { -1, 1 });

    Storage statesFilt = statesTimeStepping;
    statesFilt.pad(statesFilt.getSize() / 2);
    statesFilt.lowpassIIR(30);
    const auto statesToTrack = statesFilt.exportToTable();
    //const auto statesToTrack = statesTimeSteppingTable;
    STOFileAdapter::write(statesToTrack, "slipSolveForForce_ref_filtered.sto");


    // TODO filter statesTimeSteppingTable!
    // TODO only track coordinate values, not speeds.

    auto* stateTracking = mp.addGoal<MocoStateTrackingCost>("state_tracking");
    stateTracking->setReference(statesToTrack);

    mp.addGoal<MocoControlGoal>("effort");

    //MocoMarkerTrackingGoal markerTracking;
    //markerTracking.setName("marker_tracking");
    //MarkersReference markersRef(markersToUse);
    //markerTracking.setMarkersReference(markersRef);
    ////markerTracking.setFreeRadius(0.01);
    //markerTracking.set_weight(1);
    //mp.addGoal(markerTracking);

    // Solves in 3-4 minutes without contact tracking, 9 minutes with contact
    // tracking.
    auto* grfTracking = mp.addGoal<MocoForceTrackingCost>("grf_tracking");
    GCVSplineSet grfSplines(forcesFlattened);
    grfTracking->m_refspline_x = *grfSplines.getGCVSpline(0);
    grfTracking->m_refspline_y = *grfSplines.getGCVSpline(1);
    double normGRFs = 0.001;
    double weight = 1;
    grfTracking->set_weight(normGRFs * weight);
    grfTracking->append_forces("contact");


    MocoTropterSolver& ms = moco.initSolver();
    ms.set_multibody_dynamics_mode("implicit");
    ms.set_num_mesh_intervals(100);
    //ms.set_num_mesh_intervals(50);
    //ms.set_optim_max_iterations(2);
    // I tried setting convergence and constraint tolerances to 1e-3, and the
    // time to solve increased from 11 to 16 minutes (using EspositoMiller2018
    // for contact). Setting the tolerance to 1e-2 decreases the solve time from
    // 11 to 6 minutes. The error in the actuator force is definitely larger (up
    // to 50 N), but the match is still very good. This was all without contact
    // tracking. With contact tracking and AckermannVanDenBogert2010, the solve
    // time is about 5 minutes whether or not the convergence tolerance is set.
    // ms.set_optim_convergence_tolerance(1e-2);
    // ms.set_optim_constraint_tolerance(1e-2);
    MocoTrajectory guess = ms.createGuess();
    guess.setStatesTrajectory(statesToTrack, true);
    ms.setGuess(guess);

    MocoSolution solution = study.solve().unseal();
    solution.write("slipSolveForForce_solution.sto");
    std::cout << "RMS: " << solution.compareContinuousVariablesRMS(guess)
            << std::endl;

    SimTK::Vector springForceRecovered =
            interpolate(solution.getTime(),
                    -2500.0 * solution.getControl("actuator"),
                    springForceActualTime);
    std::cout << "Spring force error: " <<
            springForceRecovered - springForceActual << std::endl;

    // Compute the contact force for the direct collocation solution.
    const auto statesTraj = solution.exportToStatesTrajectory(mp);
    Model model = mp.getPhase().getModel();
    model.initSystem();
    const auto& contact =
            model.getComponent<StationPlaneContactForce>("contact");
    TimeSeriesTableVec3 contactForceHistory;
    contactForceHistory.setColumnLabels(
            {"SLIPcontactforce_on_station"});

    for (const auto& s : statesTraj) {
        model.realizeVelocity(s);
        contactForceHistory.appendRow(s.getTime(),
                {contact.calcContactForceOnStation(s)});
    }

    STOFileAdapter::write(contactForceHistory.flatten({"x", "y", "z"}),
            "slipSolveForForce_dircol_force.sto");

    //ms.set_num_mesh_intervals(50);
    //ms.setGuess(solution);
    //// Does not converge instantly... TODO
    //MocoSolution solution2 = study.solve().unseal();
    //solution2.write("slipSolveForForce_solution2.sto");

    //// 200 mesh intervals results in a *much* smoother solution than 50 but takes
    //// 7626 seconds to solve. and it's smoother but not smooth; very high
    //// frequency for most of the motion; smooth in the middle third.
    //ms.set_num_mesh_intervals(200);
    //ms.setGuess(solution2);
    //MocoSolution solution200 = study.solve().unseal();
    //solution200.write("slipSolveForForce_solution200.sto");

    // This works, but it takes a few minutes:
    //ms.set_num_mesh_intervals(100);
    //ms.setGuess(solution);
    //MocoSolution solution100 = study.solve().unseal();
    //solution100.write("slipSolveForForce_solution100.sto");

    // State and GRF tracking solution:
    // + Residual actuators at pelvis (rz, tx, ty)
    // + Implicit skeletal dynamics (no activation dynamics at coordinates)
    // + 100 mesh intervals
    // + AckermannVanDenBogert2010Force contact (defaults w/ stiffness = 5e5)

    study.visualize(solution);
}

int main() {


    if (false) {
        const SimTK::Real y0 = 0.5;
        // const SimTK::Real vx0 = 0.7;
        const SimTK::Real finalTime = 1.0;
        Model model = createModel();
        auto state = model.initSystem();
        model.setStateVariableValue(state, "/slider/y/value", y0);
        Manager manager(model, state);
        state = manager.integrate(finalTime);
        visualize(model, manager.getStateStorage());
    }

    // ball2d();

    // pendulum();

    //slip();

    // slip(0.05 * SimTK::Pi, -0.5);

    // TODO need to add an actuator that can rotate the leg.

    // TODO add two legs.


    // TODO cost to minimize effort.


    // TODO use a different model that has a CoordinateActuator and recover
    // the original spring force.

    // TODO inverse dynamics: solve for what the actuator force should be.

    // TODO use a lower stiffness.
    slipSolveForForce(0.05 * SimTK::Pi, -0.5);


    //    pendulumActivationCoordinateActuator();


}
