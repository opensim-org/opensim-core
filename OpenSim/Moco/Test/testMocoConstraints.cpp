/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testMocoConstraints.cpp                                      *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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

#include <simbody/internal/Constraint.h>

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Actuators/PointActuator.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/Sine.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/osimSimulation.h>

#include "Testing.h"
#include <catch2/catch_all.hpp>

using Catch::Matchers::ContainsSubstring;
using Catch::Approx;

using namespace OpenSim;
using SimTK::State;
using SimTK::UnitVec3;
using SimTK::Vec3;
using SimTK::Vector;

const int NUM_BODIES = 10;
const double BOND_LENGTH = 0.5;

/// Keep constraints satisfied to this tolerance during testing.
static const double ConstraintTol = 1e-10;

/// Compare two quantities that should have been calculated to machine tolerance
/// given the problem size, which we'll characterize by the number of mobilities
/// (borrowed from Simbody's 'testConstraints.cpp').
#define MACHINE_TEST(a, b) SimTK_TEST_EQ_SIZE(a, b, 10 * state.getNU())

/// creates a model with one sliding mass
std::unique_ptr<Model> createSlidingMassModel() {
    auto model = std::make_unique<Model>();
    model->setName("sliding_mass");
    model->set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", 10.0, SimTK::Vec3(0), SimTK::Inertia(0));
    model->addComponent(body);
    body->attachGeometry(new Sphere(0.05));

    // Allows translation along x.
    auto* joint = new SliderJoint("slider", model->getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model->addComponent(joint);

    auto* actu = new CoordinateActuator();
    actu->setCoordinate(&coord);
    actu->setName("actuator");
    actu->setOptimalForce(1);
    model->addComponent(actu);

    return model;
}

/// create a model with two sliding masses
std::unique_ptr<Model> createDoubleSlidingMassModel() {
    std::unique_ptr<Model> model = createSlidingMassModel();
    auto* body = new Body("body2", 10.0, SimTK::Vec3(0), SimTK::Inertia(0));
    model->addComponent(body);
    body->attachGeometry(new Sphere(0.05));

    // Allows translation along x.
    auto* joint = new SliderJoint("slider2", model->getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model->addComponent(joint);

    auto* actu = new CoordinateActuator();
    actu->setCoordinate(&coord);
    actu->setName("actuator2");
    actu->setOptimalForce(1);
    model->addComponent(actu);

    model->finalizeConnections();
    return model;
}

TEST_CASE("(Dummy test to support discovery in Resharper)") { REQUIRE(true); }

/// Create a model consisting of a chain of bodies. This model is nearly
/// identical to the model implemented in Simbody's 'testConstraints.cpp'.
Model createModel() {
    Model model;
    const SimTK::Real mass = 1.23;
    Body* body = new Body("body0", mass, SimTK::Vec3(0.1, 0.2, -0.03),
            mass * SimTK::UnitInertia(1.1, 1.2, 1.3, 0.01, -0.02, 0.07));
    model.addBody(body);

    GimbalJoint* joint = new GimbalJoint("joint0", model.getGround(),
            Vec3(-0.1, 0.3, 0.2), Vec3(0.3, -0.2, 0.1), *body,
            Vec3(BOND_LENGTH, 0, 0), Vec3(-0.2, 0.1, -0.3));
    model.addJoint(joint);

    for (int i = 1; i < NUM_BODIES; ++i) {
        Body& parent = model.getBodySet().get(model.getNumBodies() - 1);

        std::string bodyName = "body" + std::to_string(i + 1);
        Body* body = new Body(bodyName, mass, SimTK::Vec3(0.1, 0.2, -0.03),
                mass * SimTK::UnitInertia(1.1, 1.2, 1.3, 0.01, -0.02, 0.07));
        model.addBody(body);

        std::string jointName = "joint" + std::to_string(i + 1);
        if (i == NUM_BODIES - 5) {
            UniversalJoint* joint = new UniversalJoint(jointName, parent,
                    Vec3(-0.1, 0.3, 0.2), Vec3(0.3, -0.2, 0.1), *body,
                    Vec3(BOND_LENGTH, 0, 0), Vec3(-0.2, 0.1, -0.3));
            model.addJoint(joint);
        } else if (i == NUM_BODIES - 3) {
            BallJoint* joint = new BallJoint(jointName, parent,
                    Vec3(-0.1, 0.3, 0.2), Vec3(0.3, -0.2, 0.1), *body,
                    Vec3(BOND_LENGTH, 0, 0), Vec3(-0.2, 0.1, -0.3));
            model.addJoint(joint);
        } else {
            GimbalJoint* joint = new GimbalJoint(jointName, parent,
                    Vec3(-0.1, 0.3, 0.2), Vec3(0.3, -0.2, 0.1), *body,
                    Vec3(BOND_LENGTH, 0, 0), Vec3(-0.2, 0.1, -0.3));
            model.addJoint(joint);
        }
    }

    model.finalizeConnections();

    return model;
}

/// Create a random state for the model. This implementation mimics the random
/// state creation in Simbody's 'testConstraints.cpp'.
void createState(
        Model& model, State& state, const Vector& qOverride = Vector()) {
    state = model.initSystem();
    SimTK::Random::Uniform random;
    for (int i = 0; i < state.getNY(); ++i) state.updY()[i] = random.getValue();
    if (qOverride.size()) state.updQ() = qOverride;
    model.realizeVelocity(state);

    model.updMultibodySystem().project(state, ConstraintTol);
    model.realizeAcceleration(state);
}

/// Get model accelerations given the constraint multipliers. This calculation
/// is necessary for computing constraint defects associated with the system
/// dynamics, represented by the equations
///
///     M udot + G^T lambda + f_inertial(q,u) = f_applied
///
/// If using an explicit representation of the system dynamics, the derivatives
/// of the generalized speeds for the system need to be computed in order to
/// construct the defects. Rearranging the equations above (and noting that
/// Simbody does not actually invert the mass matrix, but rather uses an order-N
/// approach), we obtain
///
///     udot = M_inv (f_applied - f_inertial(q,u) - G^T lambda)
///          = f(q, u, lambda)
///
/// where,
///              q | generalized coordinates
///              u | generalized speeds
///         lambda | Lagrange multipliers
///
/// Since the three quantities required to compute the system accelerations
/// will eventually become NLP variables in a direct collocation problem, it is
/// not sufficient to use the internally calculated Lagrange multipliers in
/// Simbody. An intermediate calculation must be made:
///
///     f_constraint(lambda) = G^T lambda
///
/// Therefore, this method computes the generalized speed derivatives via the
/// equation
///
///     udot = M_inv (f_applied - f_inertial(q,u) - f_constraint(lambda))
///
/// Finally, note that in order for f_constraint to be used like an applied
/// force (i.e. appear on the RHS), the multipliers are negated in the call to
/// obtain Simbody constraint forces.
void calcAccelerationsFromMultipliers(const Model& model, const State& state,
        const Vector& multipliers, Vector& udot) {

    SimTK::Vector_<SimTK::SpatialVec> constraintBodyForces;
    Vector constraintMobilityForces;
    // We first need to compute the body and mobility forces associated with the
    // Lagrange multipliers provided by a solver.
    {
        const auto& matter = model.getMatterSubsystem();
        // Multipliers are negated so the constraint forces can be used like
        // applied forces.
        matter.calcConstraintForcesFromMultipliers(state, -multipliers,
                constraintBodyForces, constraintMobilityForces);
    }

    // We would like to eventually compute the model accelerations through
    // realizing to Stage::Acceleration. However, if the model has constraints,
    // realizing to Stage::Acceleration will cause Simbody to compute its own
    // Lagrange multipliers which will not necessarily be consistent with the
    // multipliers provided by a solver. Therefore, we'll first create a copy
    // of the original model, disable the its constraints, and apply the
    // constraint forces we just calculated before computing the accelerations.
    {
        // Create a copy of the original model, whose constraints we'll disable.
        Model modelDisabledConstraints = Model(model);

        // Add an OpenSim::DiscreteForces component to the new model, which
        // we'll use to the apply the constraint forces.
        DiscreteForces* constraintForces = new DiscreteForces();
        modelDisabledConstraints.addComponent(constraintForces);

        // Initialize the new model's underlying system and get a non-const
        // state, which contains slots for the original model's continuous
        // variables and new slots for the discrete variables representing the
        // constraint forces.
        SimTK::State& stateDisabledConstraints =
                modelDisabledConstraints.initSystem();
        // Update the new model's continuous variables from the passed in state.
        stateDisabledConstraints.updY() = state.getY();
        // Update the discrete forces in the new state with the constraint
        // forces we just calculated.
        constraintForces->setAllForces(stateDisabledConstraints,
                constraintMobilityForces, constraintBodyForces);

        // Disable the constraints in the new model.
        auto& matterIgnoringConstraints =
                modelDisabledConstraints.updMatterSubsystem();
        const auto NC = matterIgnoringConstraints.getNumConstraints();
        for (SimTK::ConstraintIndex cid(0); cid < NC; ++cid) {
            SimTK::Constraint& constraint =
                    matterIgnoringConstraints.updConstraint(cid);
            if (!constraint.isDisabled(stateDisabledConstraints)) {
                constraint.disable(stateDisabledConstraints);
            }
        }

        // Now we can simply realize to Stage::Acceleration on the new model to
        // get correct accelerations.
        modelDisabledConstraints.realizeAcceleration(stateDisabledConstraints);
        udot = stateDisabledConstraints.getUDot();
    }
}

/// DAE calculation subtests.
/// -------------------------
/// The following tests add a constraint to a model and check that the method
/// calcAccelerationsFromMultipliers() is implemented correctly. Each test
/// follows a similar structure:
///     1) Create a model and add a constraint between two bodies
///     2) Create a random state and realize the model to Stage::Acceleration
///     3) Check that state contains at least one Lagrange multiplier
///     4) Compute the model accelerations from Simbody
///     5) Retrieve the Lagrange multiplier values for the current state
///     6) Compute the accelerations from calcAccelerationsFromMultipliers()
///     7) Ensure that the accelerations from step 4 and 6 match
TEST_CASE("WeldConstraint", "") {
    State state;
    Model model = createModel();
    const std::string& firstBodyName =
            model.getBodySet().get(0).getAbsolutePathString();
    const std::string& lastBodyName =
            model.getBodySet().get(NUM_BODIES - 1).getAbsolutePathString();
    WeldConstraint* constraint =
            new WeldConstraint("weld", firstBodyName, lastBodyName);
    model.addConstraint(constraint);
    createState(model, state);
    // Check that constraint was added successfully.
    SimTK_TEST(state.getNMultipliers() > 0);

    const Vector& udotSimbody = model.getMatterSubsystem().getUDot(state);
    const Vector& multipliers =
            model.getMatterSubsystem().getConstraintMultipliers(state);
    Vector udotMultipliers;
    calcAccelerationsFromMultipliers(
            model, state, multipliers, udotMultipliers);
    // Check that accelerations calculated from Lagrange multipliers match
    // Simbody's accelerations.
    MACHINE_TEST(udotSimbody, udotMultipliers);
}

TEST_CASE("PointConstraint", "") {
    State state;
    Model model = createModel();
    const Body& firstBody = model.getBodySet().get(0);
    const Body& lastBody = model.getBodySet().get(NUM_BODIES - 1);
    PointConstraint* constraint =
            new PointConstraint(firstBody, Vec3(0), lastBody, Vec3(0));
    model.addConstraint(constraint);
    createState(model, state);
    // Check that constraint was added successfully.
    SimTK_TEST(state.getNMultipliers() > 0);

    const Vector& udotSimbody = model.getMatterSubsystem().getUDot(state);
    const Vector& multipliers =
            model.getMatterSubsystem().getConstraintMultipliers(state);
    Vector udotMultipliers;
    calcAccelerationsFromMultipliers(
            model, state, multipliers, udotMultipliers);
    // Check that accelerations calculated from Lagrange multipliers match
    // Simbody's accelerations.
    MACHINE_TEST(udotSimbody, udotMultipliers);
}

TEST_CASE("PointOnLineConstraint", "") {
    State state;
    Model model = createModel();
    const Body& firstBody = model.getBodySet().get(0);
    const Body& lastBody = model.getBodySet().get(NUM_BODIES - 1);
    PointOnLineConstraint* constraint = new PointOnLineConstraint(
            firstBody, Vec3(1, 0, 0), Vec3(0), lastBody, Vec3(0));
    model.addConstraint(constraint);
    createState(model, state);
    // Check that constraint was added successfully.
    SimTK_TEST(state.getNMultipliers() > 0);

    const Vector& udotSimbody = model.getMatterSubsystem().getUDot(state);
    const Vector& multipliers =
            model.getMatterSubsystem().getConstraintMultipliers(state);
    Vector udotMultipliers;
    calcAccelerationsFromMultipliers(
            model, state, multipliers, udotMultipliers);
    // Check that accelerations calculated from Lagrange multipliers match
    // Simbody's accelerations.
    MACHINE_TEST(udotSimbody, udotMultipliers);
}

TEST_CASE("ConstantDistanceConstraint", "") {
    State state;
    Model model = createModel();
    const Body& firstBody = model.getBodySet().get(0);
    const Body& lastBody = model.getBodySet().get(NUM_BODIES - 1);
    ConstantDistanceConstraint* constraint = new ConstantDistanceConstraint(
            firstBody, Vec3(0), lastBody, Vec3(0), 4.56);
    model.addConstraint(constraint);
    createState(model, state);
    // Check that constraint was added successfully.
    SimTK_TEST(state.getNMultipliers() > 0);

    const Vector& udotSimbody = model.getMatterSubsystem().getUDot(state);
    const Vector& multipliers =
            model.getMatterSubsystem().getConstraintMultipliers(state);
    Vector udotMultipliers;
    calcAccelerationsFromMultipliers(
            model, state, multipliers, udotMultipliers);
    // Check that accelerations calculated from Lagrange multipliers match
    // Simbody's accelerations.
    MACHINE_TEST(udotSimbody, udotMultipliers);
}

TEST_CASE("LockedCoordinate", "") {
    State state;
    Model model = createModel();
    CoordinateSet& coordSet = model.updCoordinateSet();
    coordSet.getLast()->set_locked(true);
    createState(model, state);
    // Check that constraint was added successfully.
    SimTK_TEST(state.getNMultipliers() > 0);

    const Vector& udotSimbody = model.getMatterSubsystem().getUDot(state);
    const Vector& multipliers =
            model.getMatterSubsystem().getConstraintMultipliers(state);
    Vector udotMultipliers;
    calcAccelerationsFromMultipliers(
            model, state, multipliers, udotMultipliers);
    // Check that accelerations calculated from Lagrange multipliers match
    // Simbody's accelerations.
    MACHINE_TEST(udotSimbody, udotMultipliers);
}

TEST_CASE("CoordinateCouplerConstraint", "") {
    State state;
    Model model = createModel();
    CoordinateSet& coordSet = model.updCoordinateSet();
    CoordinateCouplerConstraint* constraint = new CoordinateCouplerConstraint();
    Array<std::string> names;
    coordSet.getNames(names);
    constraint->setIndependentCoordinateNames(
            Array<std::string>(names.get(0), 1));
    constraint->setDependentCoordinateName(names.getLast());
    LinearFunction func(1.0, 0.0);
    constraint->setFunction(func);
    model.addConstraint(constraint);
    createState(model, state);
    // Check that constraint was added successfully.
    SimTK_TEST(state.getNMultipliers() > 0);

    const Vector& udotSimbody = model.getMatterSubsystem().getUDot(state);
    const Vector& multipliers =
            model.getMatterSubsystem().getConstraintMultipliers(state);
    Vector udotMultipliers;
    calcAccelerationsFromMultipliers(
            model, state, multipliers, udotMultipliers);
    // Check that accelerations calculated from Lagrange multipliers match
    // Simbody's accelerations.
    MACHINE_TEST(udotSimbody, udotMultipliers);
}

TEST_CASE("PrescribedMotion", "") {
    State state;
    Model model = createModel();
    CoordinateSet& coordSet = model.updCoordinateSet();
    LinearFunction func(1.0, 0.0);
    coordSet.getLast()->setPrescribedFunction(func);
    coordSet.getLast()->setDefaultIsPrescribed(true);
    createState(model, state);
    // Check that constraint was added successfully.
    SimTK_TEST(state.getNMultipliers() > 0);

    const Vector& udotSimbody = model.getMatterSubsystem().getUDot(state);
    const Vector& multipliers =
            model.getMatterSubsystem().getConstraintMultipliers(state);
    Vector udotMultipliers;
    calcAccelerationsFromMultipliers(
            model, state, multipliers, udotMultipliers);
    // Check that accelerations calculated from Lagrange multipliers match
    // Simbody's accelerations.
    MACHINE_TEST(udotSimbody, udotMultipliers);
}

/// Create a torque-actuated double pendulum model. Each subtest will add to the
/// model the relevant constraint(s).
std::unique_ptr<Model> createDoublePendulumModel() {
    auto model = std::make_unique<Model>();
    model->setName("double_pendulum");

    using SimTK::Inertia;
    using SimTK::Vec3;

    // Create two links, each with a mass of 1 kg, center of mass at the body's
    // origin, and moments and products of inertia of zero.
    auto* b0 = new OpenSim::Body("b0", 1, Vec3(0), Inertia(1));
    model->addBody(b0);
    auto* b1 = new OpenSim::Body("b1", 1, Vec3(0), Inertia(1));
    model->addBody(b1);

    // Add station representing the model end-effector.
    auto* endeff = new Station(*b1, Vec3(0));
    endeff->setName("endeff");
    model->addComponent(endeff);

    // Connect the bodies with pin joints. Assume each body is 1 m long.
    auto* j0 = new PinJoint("j0", model->getGround(), Vec3(0), Vec3(0), *b0,
            Vec3(-1, 0, 0), Vec3(0));
    auto& q0 = j0->updCoordinate();
    q0.setName("q0");
    q0.setDefaultValue(0);
    auto* j1 = new PinJoint(
            "j1", *b0, Vec3(0), Vec3(0), *b1, Vec3(-1, 0, 0), Vec3(0));
    auto& q1 = j1->updCoordinate();
    q1.setName("q1");
    q1.setDefaultValue(SimTK::Pi);
    model->addJoint(j0);
    model->addJoint(j1);

    // Add coordinate actuators.
    auto* tau0 = new CoordinateActuator();
    tau0->setCoordinate(&j0->updCoordinate());
    tau0->setName("tau0");
    tau0->setOptimalForce(1);
    model->addComponent(tau0);
    auto* tau1 = new CoordinateActuator();
    tau1->setCoordinate(&j1->updCoordinate());
    tau1->setName("tau1");
    tau1->setOptimalForce(1);
    model->addComponent(tau1);

    // Add display geometry.
    Ellipsoid bodyGeometry(0.5, 0.1, 0.1);
    SimTK::Transform transform(SimTK::Vec3(-0.5, 0, 0));
    auto* b0Center = new PhysicalOffsetFrame("b0_center", *b0, transform);
    b0->addComponent(b0Center);
    b0Center->attachGeometry(bodyGeometry.clone());
    auto* b1Center = new PhysicalOffsetFrame("b1_center", *b1, transform);
    b1->addComponent(b1Center);
    b1Center->attachGeometry(bodyGeometry.clone());

    return model;
}

/// Run a forward simulation using controls from an OCP solution and compare the
/// state trajectories.
MocoTrajectory runForwardSimulation(
        Model model, const MocoSolution& solution, const double& tol) {

    // Get actuator names.
    model.initSystem();
    OpenSim::Array<std::string> actuNames;
    const auto modelPath = model.getAbsolutePath();
    for (const auto& actu : model.getComponentList<Actuator>()) {
        actuNames.append(actu.getAbsolutePathString());
    }

    // Add prescribed controllers to actuators in the model, where the control
    // functions are splined versions of the actuator controls from the OCP
    // solution.
    const SimTK::Vector& time = solution.getTime();
    auto* controller = new PrescribedController();
    controller->setName("prescribed_controller");
    for (int i = 0; i < actuNames.size(); ++i) {
        const auto control = solution.getControl(actuNames[i]);
        GCVSpline controlFunction(5, time.nrow(), &time[0], &control[0]);
        const auto& actu = model.getComponent<Actuator>(actuNames[i]);
        controller->addActuator(actu);
        controller->prescribeControlForActuator(
                actu.getName(), controlFunction);
    }
    model.addController(controller);

    // Add states reporter to the model.
    auto* statesRep = new StatesTrajectoryReporter();
    statesRep->setName("states_reporter");
    statesRep->set_report_time_interval(0.001);
    model.addComponent(statesRep);

    // Add a TableReporter to collect the controls.
    auto* controlsRep = new TableReporter();
    for (int i = 0; i < actuNames.size(); ++i) {
        controlsRep->addToReport(
                model.getComponent(actuNames[i]).getOutput("actuation"),
                actuNames[i]);
    }
    model.addComponent(controlsRep);

    // Simulate!
    SimTK::State state = model.initSystem();
    state.setTime(time[0]);
    Manager manager(model);
    manager.getIntegrator().setAccuracy(1e-9);
    manager.initialize(state);
    state = manager.integrate(time[time.size() - 1]);

    // Export results from states reporter to a TimeSeries Table
    TimeSeriesTable states;
    states = statesRep->getStates().exportToTable(model);

    TimeSeriesTable controls;
    controls = controlsRep->getTable();

    // Create a MocoTrajectory to facilitate states trajectory comparison (with
    // dummy data for the multipliers, which we'll ignore).
    const auto& statesTimes = states.getIndependentColumn();
    SimTK::Vector timeVec((int)statesTimes.size(), statesTimes.data(), true);
    auto forwardSolution = MocoTrajectory(timeVec, states.getColumnLabels(),
            controls.getColumnLabels(), states.getColumnLabels(), {},
            states.getMatrix(), controls.getMatrix(), states.getMatrix(),
            SimTK::RowVector(0));

    // Compare controls between foward simulation and OCP solution. These
    // should match very closely, since the foward simulation controls are
    // created from splines of the OCP solution controls
    SimTK_TEST_EQ_TOL(solution.compareContinuousVariablesRMS(
                              forwardSolution, {{"controls", {}}}),
            0, 1e-9);

    // Compare states trajectory between forward simulation and OCP solution.
    // The states trajectory may not match as well as the controls.
    SimTK_TEST_EQ_TOL(solution.compareContinuousVariablesRMS(
                              forwardSolution, {{"states", {}}}),
            0, tol);

    return forwardSolution;
}

// Check that the constraint errors from a MocoSolution are within a specified
// tolerance. Also, check that the slack variables have a reasonable magnitude.
void checkConstraintErrors(const MocoSolution& solution, const Model& model,
        bool enforce_constraint_derivatives, const std::string& method) {
    StatesTrajectory statesTraj = solution.exportToStatesTrajectory(model);
    for (int i = 0; i < (int)statesTraj.getSize(); ++i) {
        const auto& s = statesTraj.get(i);
        model.realizeAcceleration(s);
        const auto& qerr = s.getQErr();
        const auto& uerr = s.getUErr();
        const auto& udoterr = s.getUDotErr();

        // If we're using the Posa et al. 2016 method and not enforcing
        // constraint derivatives, we'll loosen the tolerance for the constraint
        // at the midpoint of the mesh intervals, since we do not explicitly 
        // enforce the constraints at these points.
        bool loosen_tol = !enforce_constraint_derivatives &&
                          method == "Posa2016" &&
                          i % 2 != 0;
        double tol = loosen_tol ? 1e-3 : 1e-6;
        for (int j = 0; j < qerr.size(); ++j) {
            REQUIRE_THAT(qerr[j], Catch::Matchers::WithinAbs(0, tol));
        }
        if (enforce_constraint_derivatives) {
            for (int j = 0; j < uerr.size(); ++j) {
                REQUIRE_THAT(uerr[j], Catch::Matchers::WithinAbs(0, tol));
            }
            for (int j = 0; j < udoterr.size(); ++j) {
                REQUIRE_THAT(udoterr[j], Catch::Matchers::WithinAbs(0, tol));
            }
        }
    }

    // If problems have converged and all kinematic constraints are satisfied, 
    // we expect the slack variables to be reasonably small.
    const auto& slacks = solution.getSlacksTrajectory();
    for (int icol = 0; icol < slacks.ncol(); ++icol) {
        CAPTURE(slacks.col(icol));
        REQUIRE_THAT(SimTK::max(SimTK::abs(slacks.col(icol))), 
                Catch::Matchers::WithinAbs(0, 1e-5));
    }
}

/// Direct collocation subtests.
/// ----------------------------

/// Solve an optimal control problem where a double pendulum must reach a
/// specified final configuration while subject to a constraint that its
/// end-effector must lie on a vertical line through the origin and minimize
/// control effort.
template <typename TestType>
void testDoublePendulumPointOnLine(
        bool enforce_constraint_derivatives, std::string dynamics_mode,
        const std::string& kinematic_constraint_method = "Posa2016",
        const std::string& transcription_scheme = "hermite-simpson",
        int num_mesh_intervals = 20) {
    MocoStudy study;
    study.setName("double_pendulum_point_on_line");
    MocoProblem& mp = study.updProblem();
    // Create double pendulum model and add the point-on-line constraint. The
    // constraint consists of a vertical line in the y-direction (defined in
    // ground) and the model end-effector point (the origin of body "b1").
    auto model = createDoublePendulumModel();
    const Body& b1 = model->getBodySet().get("b1");
    const Station& endeff = model->getComponent<Station>("endeff");

    PointOnLineConstraint* constraint =
            new PointOnLineConstraint(model->getGround(), Vec3(0, 1, 0),
                    Vec3(0), b1, endeff.get_location());
    model->addConstraint(constraint);
    model->finalizeConnections();
    mp.setModelAsCopy(*model);

    mp.setTimeBounds(0, 1);
    // Coordinate value state boundary conditions are consistent with the
    // point-on-line constraint.
    const double theta_i = 0.5;
    const double theta_f = SimTK::Pi / 2;
    mp.setStateInfo(
            "/jointset/j0/q0/value", {theta_i, theta_f}, theta_i, theta_f);
    mp.setStateInfo("/jointset/j0/q0/speed", {-50, 50});
    {
        double initial = SimTK::Pi - 2 * theta_i;
        double final = SimTK::Pi - 2 * theta_f;
        mp.setStateInfo(
                "/jointset/j1/q1/value", {final, initial}, initial, final);
    }
    mp.setStateInfo("/jointset/j1/q1/speed", {-50, 50});
    mp.setControlInfo("/tau0", {-100, 100});
    mp.setControlInfo("/tau1", {-100, 100});

    auto* effort = mp.addGoal<MocoControlGoal>();

    auto& ms = study.initSolver<TestType>();
    ms.set_num_mesh_intervals(num_mesh_intervals);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-3);
    ms.set_transcription_scheme(transcription_scheme);
    ms.set_enforce_constraint_derivatives(enforce_constraint_derivatives);
    ms.set_kinematic_constraint_method(kinematic_constraint_method);
    if (!enforce_constraint_derivatives) {
        ms.set_minimize_lagrange_multipliers(true);
        ms.set_lagrange_multiplier_weight(10);
    }
    ms.set_multibody_dynamics_mode(dynamics_mode);
    ms.setGuess("bounds");

    MocoSolution solution = study.solve();
    solution.write("testConstraints_testDoublePendulumPointOnLine.sto");
    // moco.visualize(solution);

    // Check that the constraint errors are within a specified tolerance.
    model->initSystem();
    checkConstraintErrors(solution, *model, enforce_constraint_derivatives,
            kinematic_constraint_method);

    // Check that the end-effector point lies on the vertical line through the
    // origin.
    model->initSystem();
    StatesTrajectory states = solution.exportToStatesTrajectory(mp);
    for (int i = 0; i < (int)states.getSize(); ++i) {
        const auto& s = states.get(i);
        model->realizePosition(s);
        const SimTK::Vec3& loc = endeff.getLocationInGround(s);

        // The end-effector should not have moved in the x- or z-directions.
        SimTK_TEST_EQ_TOL(loc[0], 0, 1e-2);
        SimTK_TEST_EQ_TOL(loc[2], 0, 1e-2);
    }

    // Run a forward simulation using the solution controls in prescribed
    // controllers for the model actuators and see if we get the correct states
    // trajectory back.
    runForwardSimulation(*model, solution, 2);
}

/// Solve an optimal control problem where a double pendulum must reach a
/// specified final configuration while subject to a constraint that couples
/// its two coordinates together via a linear relationship and minimizing
/// control effort.
template <typename SolverType>
void testDoublePendulumCoordinateCoupler(
        bool enforce_constraint_derivatives, std::string dynamics_mode,
        std::string kinematic_constraint_method = "Posa2016",
        std::string transcription_scheme = "hermite-simpson",
        int num_mesh_intervals = 20) {
    MocoStudy study;
    study.setName("double_pendulum_coordinate_coupler");

    // Create double pendulum model and add the coordinate coupler constraint.
    auto model = createDoublePendulumModel();
    const Coordinate& q0 = model->getCoordinateSet().get("q0");
    const Coordinate& q1 = model->getCoordinateSet().get("q1");
    CoordinateCouplerConstraint* constraint = new CoordinateCouplerConstraint();
    Array<std::string> indepCoordNames;
    indepCoordNames.append("q0");
    constraint->setIndependentCoordinateNames(indepCoordNames);
    constraint->setDependentCoordinateName("q1");
    // Represented by the following equation,
    //      q1 = m*q0 + b
    // this linear function couples the two model coordinates such that given
    // the boundary conditions for q0 from testDoublePendulumPointOnLine, the
    // same boundary conditions for q1 should be achieved without imposing
    // bounds for this coordinate.
    const SimTK::Real m = -2;
    const SimTK::Real b = SimTK::Pi;
    LinearFunction linFunc(m, b);
    // Avoid CoordinateCoupler::setFunction(const Function&); it has a leak.
    constraint->setFunction(&linFunc);
    model->addConstraint(constraint);
    model->finalizeConnections();

    MocoProblem& mp = study.updProblem();
    mp.setModelAsCopy(*model);
    mp.setTimeBounds(0, 1);
    // Boundary conditions are only enforced for the first coordinate, so we can
    // test that the second coordinate is properly coupled.
    mp.setStateInfo("/jointset/j0/q0/value", {-5, 5}, 0, SimTK::Pi / 2);
    mp.setStateInfo("/jointset/j0/q0/speed", {-10, 10}, 0, 0);
    mp.setStateInfo("/jointset/j1/q1/value", {-10, 10});
    mp.setStateInfo("/jointset/j1/q1/speed", {-5, 5}, 0, 0);
    mp.setControlInfo("/tau0", {-50, 50});
    mp.setControlInfo("/tau1", {-50, 50});
    auto* effort = mp.addGoal<MocoControlGoal>();

    auto& ms = study.initSolver<SolverType>();
    ms.set_num_mesh_intervals(num_mesh_intervals);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-3);
    ms.set_transcription_scheme(transcription_scheme);
    ms.set_enforce_constraint_derivatives(enforce_constraint_derivatives);
    ms.set_kinematic_constraint_method(kinematic_constraint_method);
    if (!enforce_constraint_derivatives) {
        ms.set_minimize_lagrange_multipliers(true);
        ms.set_lagrange_multiplier_weight(10);
    }
    ms.set_multibody_dynamics_mode(dynamics_mode);
    ms.setGuess("bounds");

    MocoSolution solution = study.solve();
    solution.write("testConstraints_testDoublePendulumCoordinateCoupler.sto");
    //study.visualize(solution);

    // Check that the constraint errors are within a specified tolerance.
    model->initSystem();
    checkConstraintErrors(solution, *model, enforce_constraint_derivatives,
            kinematic_constraint_method);

    // Check that the coordinates are coupled according to the linear function.
    StatesTrajectory states = solution.exportToStatesTrajectory(mp);
    for (int i = 0; i < (int)states.getSize(); ++i) {
        const auto& s = states.get(i);
        model->realizePosition(s);

        // The coordinates should be coupled according to the linear function
        // described above.
        SimTK_TEST_EQ_TOL(q1.getValue(s), m * q0.getValue(s) + b, 1e-2);
    }

    // Run a forward simulation using the solution controls in prescribed
    // controllers for the model actuators and see if we get the correct states
    // trajectory back.
    runForwardSimulation(*model, solution, 1e-1);
}

/// Solve an optimal control problem where a double pendulum must follow a
/// prescribed motion based on sinusoidal functions for each coordinate.
template <typename SolverType>
void testDoublePendulumPrescribedMotion(
        bool enforce_constraint_derivatives,
        std::string dynamics_mode, 
        std::string kinematic_constraint_method = "Posa2016",
        std::string transcription_scheme = "hermite-simpson",
        int num_mesh_intervals = 20) {
    MocoStudy study;
    study.setName("double_pendulum_prescribed_motion");
    MocoProblem& mp = study.updProblem();

    // Create double pendulum model.
    auto model = createDoublePendulumModel();
    model->initSystem();

    // Apply the prescribed motion constraints.
    Sine q0func(2.0, 1.0, 0.0);
    Coordinate& q0 = model->updJointSet().get("j0").updCoordinate();
    q0.setPrescribedFunction(q0func);
    q0.setDefaultIsPrescribed(true);
    Sine q1func(0.5, 2.0, 0.5*SimTK::Pi);
    Coordinate& q1 = model->updJointSet().get("j1").updCoordinate();
    q1.setPrescribedFunction(q1func);
    q1.setDefaultIsPrescribed(true);

    // Set the model after implementing the constraints.
    mp.setModelAsCopy(*model);

    mp.setTimeBounds(0, 1);
    mp.setStateInfo("/jointset/j0/q0/value", {-10, 10});
    mp.setStateInfo("/jointset/j0/q0/speed", {-50, 50});
    mp.setStateInfo("/jointset/j1/q1/value", {-10, 10});
    mp.setStateInfo("/jointset/j1/q1/speed", {-50, 50});
    mp.setControlInfo("/tau0", {-25, 25});
    mp.setControlInfo("/tau1", {-25, 25});

    auto* effort = mp.addGoal<MocoControlGoal>();

    auto& ms = study.initSolver<SolverType>();
    ms.set_num_mesh_intervals(num_mesh_intervals);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-3);
    ms.set_transcription_scheme(transcription_scheme);
    ms.set_enforce_constraint_derivatives(enforce_constraint_derivatives);
    ms.set_kinematic_constraint_method(kinematic_constraint_method);
    if (!enforce_constraint_derivatives) {
        ms.set_minimize_lagrange_multipliers(true);
        ms.set_lagrange_multiplier_weight(10);
    }
    ms.set_multibody_dynamics_mode(dynamics_mode);

    MocoSolution solution = study.solve().unseal();
    solution.write("testConstraints_testDoublePendulumPrescribedMotion.sto");
    //study.visualize(solution);

    // Check that the constraint errors are within a specified tolerance.
    model->initSystem();
    checkConstraintErrors(solution, *model, enforce_constraint_derivatives,
            kinematic_constraint_method);

    // Run a forward simulation using the solution controls in prescribed
    // controllers for the model actuators and see if we get the correct states
    // trajectory back.
    runForwardSimulation(*model, solution, 1e-1);
}

TEST_CASE("DoublePendulum tests, Posa2016 method - MocoCasADiSolver", 
        "[casadi]") {
    bool enforce_constraint_derivatives = GENERATE(true, false);
    std::string dynamics_mode = GENERATE(as<std::string>{}, 
            "explicit", "implicit");
    std::string section = fmt::format(
            "enforce derivatives: {}, dynamics_mode: {}", 
            enforce_constraint_derivatives, dynamics_mode);

    DYNAMIC_SECTION(section) {
        SECTION("CoordinateCouplerConstraint") {
            testDoublePendulumCoordinateCoupler<MocoCasADiSolver>(
                    enforce_constraint_derivatives, dynamics_mode, "Posa2016");
        }
        SECTION("PrescribedMotion") {
            testDoublePendulumPrescribedMotion<MocoCasADiSolver>(
                    enforce_constraint_derivatives, dynamics_mode, "Posa2016");
        }
        SECTION("PointOnLine") {
            testDoublePendulumPointOnLine<MocoCasADiSolver>(
                    enforce_constraint_derivatives, dynamics_mode, "Posa2016");
        }
    }
}

TEST_CASE("DoublePendulum tests, Bordalba2023 method", "[casadi]") {
    std::string scheme = GENERATE(as<std::string>{},
            "trapezoidal", "hermite-simpson", "legendre-gauss-3", 
            "legendre-gauss-radau-3");
    std::string dynamics_mode = GENERATE(as<std::string>{}, 
            "explicit", "implicit");
    std::string section = fmt::format("scheme: {}, dynamics_mode: {}", 
            scheme, dynamics_mode);

    // Trapezoidal rule requires more mesh intervals to keep slack variables 
    // small.
    int num_mesh_intervals = scheme == "trapezoidal" ? 50 : 25;
    DYNAMIC_SECTION(section) {
        SECTION("CoordinateCouplerConstraint") {
            testDoublePendulumCoordinateCoupler<MocoCasADiSolver>(true, 
                    dynamics_mode, "Bordalba2023", scheme, num_mesh_intervals);
        }
        SECTION("PrescribedMotion") {
            testDoublePendulumPrescribedMotion<MocoCasADiSolver>(true, 
                    dynamics_mode, "Bordalba2023", scheme, num_mesh_intervals);
        }
        SECTION("PointOnLine") {
            testDoublePendulumPointOnLine<MocoCasADiSolver>(
                    true, dynamics_mode, "Bordalba2023", scheme, 
                    num_mesh_intervals);
        }
    }
}

TEST_CASE("Bad configurations with kinematic constraints") {
    MocoStudy study;
    study.setName("double_pendulum_coordinate_coupler");

    // Create a double pendulum model and add a constraint.
    auto model = createDoublePendulumModel();
    const Coordinate& q0 = model->getCoordinateSet().get("q0");
    const Coordinate& q1 = model->getCoordinateSet().get("q1");
    CoordinateCouplerConstraint* constraint = new CoordinateCouplerConstraint();
    Array<std::string> indepCoordNames;
    indepCoordNames.append("q0");
    constraint->setIndependentCoordinateNames(indepCoordNames);
    constraint->setDependentCoordinateName("q1");
    const SimTK::Real m = -2;
    const SimTK::Real b = SimTK::Pi;
    LinearFunction linFunc(m, b);
    constraint->setFunction(&linFunc);
    model->addConstraint(constraint);
    model->finalizeConnections();
    MocoProblem& mp = study.updProblem();
    mp.setModelAsCopy(*model);
    mp.setTimeBounds(0, 1);

    auto& ms = study.initSolver<MocoCasADiSolver>();

    SECTION("Enforce constraint derivatives") {
        ms.set_enforce_constraint_derivatives(false);
        ms.set_kinematic_constraint_method("Bordalba2023");
        CHECK_THROWS_WITH(study.solve(),
                ContainsSubstring(
                        "The Bordalba et al. 2023 method for enforcing"));
    }

    SECTION("Posa2016 method with Hermite-Simpson only") {
        ms.set_enforce_constraint_derivatives(true);
        ms.set_kinematic_constraint_method("Posa2016");
        ms.set_transcription_scheme("trapezoidal");
        CHECK_THROWS_WITH(study.solve(),
                ContainsSubstring(
                        "Expected the 'hermite-simpson' transcription scheme"));
    }
}



class EqualControlConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(EqualControlConstraint, MocoPathConstraint);

protected:
    void initializeOnModelImpl(
            const Model& model, const MocoProblemInfo&) const override {
        // Make sure the model generates a state object with the two controls we
        // expect, no more and no less.
        const auto state = model.getWorkingState();
        model.realizeVelocity(state);
        OPENSIM_THROW_IF(model.getControls(state).size() != 2, Exception,
                "State has incorrect number of controls (two expected).");

        // There is only constraint equation: match the two model controls.
        setNumEquations(1);
    }
    void calcPathConstraintErrorsImpl(
            const SimTK::State& state, SimTK::Vector& errors) const override {
        getModel().realizeVelocity(state);

        const auto& controls = getModel().getControls(state);
        // In the problem below, the actuators are bilateral and act in
        // opposite directions, so we use addition to create the residual here.
        errors[0] = controls[1] + controls[0];
    }
};

/// Solve an optimal control problem where a double pendulum must reach a
/// specified final configuration while subject to a constraint that its
/// actuators must produce an equal control trajectory.
TEMPLATE_TEST_CASE("DoublePendulumEqualControl", "",
        MocoCasADiSolver) {
    OpenSim::Object::registerType(EqualControlConstraint());
    MocoStudy study;
    study.setName("double_pendulum_equal_control");
    MocoProblem& mp = study.updProblem();
    auto model = createDoublePendulumModel();
    model->finalizeConnections();
    mp.setModelAsCopy(*model);

    auto* equalControlConstraint =
            mp.addPathConstraint<EqualControlConstraint>();
    MocoConstraintInfo cInfo;
    cInfo.setBounds(std::vector<MocoBounds>(1, {0, 0}));
    equalControlConstraint->setConstraintInfo(cInfo);

    mp.setTimeBounds(0, 1);
    // Coordinate value state boundary conditions are consistent with the
    // point-on-line constraint and should require the model to "unfold" itself.
    mp.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0, SimTK::Pi / 2);
    mp.setStateInfo("/jointset/j0/q0/speed", {-50, 50});
    mp.setStateInfo("/jointset/j1/q1/value", {-10, 10}, SimTK::Pi, 0);
    mp.setStateInfo("/jointset/j1/q1/speed", {-50, 50});
    mp.setControlInfo("/tau0", {-50, 50});
    mp.setControlInfo("/tau1", {-50, 50});

    mp.addGoal<MocoControlGoal>();

    auto& ms = study.initSolver<TestType>();
    ms.set_num_mesh_intervals(25);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-3);
    ms.setGuess("bounds");

    MocoSolution solution = study.solve();
    solution.write("testConstraints_testDoublePendulumEqualControl.sto");
    // moco.visualize(solution);

    const auto& control_tau0 = solution.getControl("/tau0");
    const auto& control_tau1 = solution.getControl("/tau1");
    const auto& control_res = control_tau1.abs() - control_tau0.abs();
    SimTK_TEST_EQ_TOL(control_res.normRMS(), 0, 1e-6);

    // Run a forward simulation using the solution controls in prescribed
    // controllers for the model actuators and see if we get the correct states
    // trajectory back.
    // TODO why does the forward solution match so poorly here?
    MocoTrajectory forwardSolution = runForwardSimulation(*model, solution, 2);
    // moco.visualize(forwardSolution);

    // Test de/serialization.
    // ======================
    std::string setup_fname =
            "testConstraints_testDoublePendulumEqualControl.omoco";
    study.print(setup_fname);
    MocoSolution solutionDeserialized;
    MocoStudy mocoDeserialize(setup_fname);
    solutionDeserialized = mocoDeserialize.solve();
    SimTK_TEST(solution.isNumericallyEqual(solutionDeserialized));
}

/// Test that a problem that fails with path constraints does not return a zero
/// time vector.
TEMPLATE_TEST_CASE("FailWithPathConstraints", "",
        MocoCasADiSolver) {
    OpenSim::Object::registerType(EqualControlConstraint());
    MocoStudy study;
    study.setName("path_constraint_fail");
    MocoProblem& mp = study.updProblem();
    auto model = createDoublePendulumModel();
    model->finalizeConnections();
    mp.setModelAsCopy(*model);
    auto* equalControlConstraint =
            mp.addPathConstraint<EqualControlConstraint>();
    MocoConstraintInfo cInfo;
    cInfo.setBounds(std::vector<MocoBounds>(1, {0, 0}));
    equalControlConstraint->setConstraintInfo(cInfo);
    mp.setTimeBounds(0, 1);
    mp.addGoal<MocoControlGoal>();
    auto& ms = study.initSolver<TestType>();
    ms.set_optim_max_iterations(1);
    ms.set_num_mesh_intervals(5);
    ms.set_verbosity(2);
    MocoSolution solution = study.solve().unseal();
}

class MocoJointReactionComponentGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoJointReactionComponentGoal, MocoGoal);

public:
    void initializeOnModelImpl(const Model&) const override {
        setRequirements(1, 1);
    }
    void calcIntegrandImpl(const IntegrandInput& input,
            SimTK::Real& integrand) const override {
        getModel().realizeAcceleration(input.state);
        const auto& joint = getModel().getComponent<Joint>("jointset/j1");
        // Minus sign since we are maximizing.
        integrand =
                -joint.calcReactionOnChildExpressedInGround(input.state)[0][0];
    }
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.integral;
    }
};

template <typename TestType>
void testDoublePendulumJointReactionGoal(std::string dynamics_mode) {
    MocoStudy study;
    study.setName("double_pendulum");
    MocoProblem& mp = study.updProblem();
    // Create double pendulum model and add prescribed motion constraints.
    const auto pi = SimTK::Pi;
    auto model = createDoublePendulumModel();
    model->updCoordinateSet().get("q0").setPrescribedFunction(
            Constant(0.25 * pi));
    model->updCoordinateSet().get("q0").setDefaultIsPrescribed(true);
    model->updCoordinateSet().get("q1").setPrescribedFunction(
            Constant(0.5 * pi));
    model->updCoordinateSet().get("q1").setDefaultIsPrescribed(true);

    const Station& endeff = model->getComponent<Station>("endeff");
    auto* actuator = new PointActuator("b1");
    actuator->setName("push");
    actuator->set_point(endeff.get_location());
    actuator->set_point_is_global(false);
    actuator->set_direction(Vec3(0, 0, -1));
    actuator->set_force_is_global(true);
    model->addComponent(actuator);

    model->finalizeConnections();
    mp.setModelAsCopy(*model);

    mp.setTimeBounds(0, 1);
    mp.setStateInfo("/jointset/j0/q0/value", {-0.6 * pi, 0.6 * pi});
    mp.setStateInfo("/jointset/j0/q0/speed", {-10, 10});
    mp.setStateInfo("/jointset/j1/q1/value", {0, pi});
    mp.setStateInfo("/jointset/j1/q1/speed", {-10, 10});
    mp.setControlInfo("/tau0", {-20, 20});
    mp.setControlInfo("/tau1", {-20, 20});
    mp.setControlInfo("/push", {-20, 20});

    // This cost tries to *maximize* joint j1's reaction torque in the
    // x-direction, which should cause the actuator "push" to hit its upper
    // bound.
    mp.addGoal<MocoJointReactionComponentGoal>();

    auto& ms = study.initSolver<TestType>();
    int N = 5;
    ms.set_num_mesh_intervals(N);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-6);
    ms.set_transcription_scheme("hermite-simpson");
    ms.set_enforce_constraint_derivatives(true);
    ms.set_minimize_lagrange_multipliers(true);
    ms.set_lagrange_multiplier_weight(10);
    ms.set_multibody_dynamics_mode(dynamics_mode);
    ms.setGuess("bounds");

    MocoSolution solution = study.solve().unseal();
    solution.write("testConstraints_testDoublePendulumJointReactionGoal.sto");

    // Check that the actuator "push" is hitting its upper bound.
    CHECK(solution.getControl("/push")[0] == Approx(20).epsilon(1e-4));
    // Check that j1's x-direction reaction torque (the only objective term)
    // is the proper value.
    CHECK(solution.getObjective() == Approx(-1. / sqrt(2) * 20).epsilon(1e-2));
}

TEMPLATE_TEST_CASE("Joint reactions w/ constraint derivatives",
        "[explicit]", MocoCasADiSolver) {
    testDoublePendulumJointReactionGoal<TestType>("explicit");
}

TEMPLATE_TEST_CASE("Joint reactions implicit w/ constraint derivatives",
        "[implicit][casadi]", MocoCasADiSolver) {
    testDoublePendulumJointReactionGoal<TestType>("implicit");
}

struct AccelerationsAndJointReaction {
    SimTK::Vector udot;
    SimTK::SpatialVec reaction;
};

// TODO: Don't have direct access to multipliers. We need to know how to
// translate multipliers into joint reactions.
class MocoMultiplierAccelerationGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoMultiplierAccelerationGoal, MocoGoal);

public:
    MocoMultiplierAccelerationGoal(std::string jointName,
            std::shared_ptr<AccelerationsAndJointReaction> data)
            : m_jointName(jointName), m_data(data) {}
    void initializeOnModelImpl(const Model&) const override {
        setRequirements(0, 1);
    }
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        const auto& state = input.final_state;
        getModel().realizeAcceleration(state);
        m_data->udot = input.final_state.getUDot();
        const auto& joint = getModel().getComponent<Joint>(m_jointName);
        m_data->reaction =
                joint.calcReactionOnChildExpressedInGround(input.final_state);

        cost[0] = 0;
    }

private:
    std::string m_jointName;
    std::shared_ptr<AccelerationsAndJointReaction> m_data;
};

// Normally, Simbody computes its own generalized accelerations and Lagrange
// multipliers. In Moco, when a model has kinematic constraints, we provide
// separate multipliers. When using implicit dynamics, we provide separate
// accelerations. This test ensures that these accelerations and multipliers are
// those that Moco specified, not what Simbody computes.
TEST_CASE("Goals use Moco-defined accelerations and multipliers", "[casadi]") {
    MocoStudy study;
    study.setName("mass_welded");
    MocoProblem& problem = study.updProblem();
    // Rigid body welded to ground.
    Model model;
    const double mass = 1.0;
    auto* body = new Body("body", mass, SimTK::Vec3(0), SimTK::Inertia(1));
    model.addBody(body);

    auto* joint = new PlanarJoint("joint", model.getGround(), *body);
    joint->updCoordinate(PlanarJoint::Coord::RotationZ).setName("rz");
    joint->updCoordinate(PlanarJoint::Coord::TranslationX).setName("tx");
    joint->updCoordinate(PlanarJoint::Coord::TranslationY).setName("ty");
    model.addJoint(joint);

    auto* constr = new WeldConstraint("constraint", model.getGround(),
            SimTK::Transform(), *body, SimTK::Transform());
    model.addConstraint(constr);
    model.finalizeConnections();
    problem.setModelAsCopy(model);

    problem.setTimeBounds(0, 1);

    // This goal is used to populate the testData struct, which will contain
    // the accelerations and joint reactions generated by Moco.
    auto testData = std::make_shared<AccelerationsAndJointReaction>();
    problem.addGoal<MocoMultiplierAccelerationGoal>("jointset/joint", testData);

    auto& solver = study.initCasADiSolver();
    int N = 5;
    solver.set_num_mesh_intervals(N);
    solver.set_minimize_lagrange_multipliers(true);
    solver.set_multibody_dynamics_mode("implicit");
    solver.set_optim_max_iterations(0);
    solver.set_parallel(0);
    auto guess = solver.createGuess("bounds");
    int Nguess = 2;
    guess.resampleWithNumTimes(Nguess);

    // Set positions to zero.
    const double rz = 0;
    guess.setState(
            "/jointset/joint/rz/value", createVectorLinspace(Nguess, rz, rz));
    const double tx = 0;
    guess.setState(
            "/jointset/joint/tx/value", createVectorLinspace(Nguess, tx, tx));
    const double ty = 0;
    guess.setState(
            "/jointset/joint/ty/value", createVectorLinspace(Nguess, ty, ty));

    // Set speeds to "random" values.
    const double rz_u = 2.471;
    guess.setState("/jointset/joint/rz/speed",
            createVectorLinspace(Nguess, rz_u, rz_u));
    const double tx_u = -1.61;
    guess.setState("/jointset/joint/tx/speed",
            createVectorLinspace(Nguess, tx_u, tx_u));
    const double ty_u = 6.0162;
    guess.setState("/jointset/joint/ty/speed",
            createVectorLinspace(Nguess, ty_u, ty_u));

    // Set multpliers to "random" values.
    const double lambda0 = 0.813604;
    guess.setMultiplier(
            "lambda_cid3_p0", createVectorLinspace(Nguess, lambda0, lambda0));
    const double lambda1 = 1.390461;
    guess.setMultiplier(
            "lambda_cid3_p1", createVectorLinspace(Nguess, lambda1, lambda1));
    const double lambda2 = 0.614711;
    guess.setMultiplier(
            "lambda_cid3_p2", createVectorLinspace(Nguess, lambda2, lambda2));
    const double lambda3 = 7.246214;
    guess.setMultiplier(
            "lambda_cid3_p3", createVectorLinspace(Nguess, lambda3, lambda3));
    const double lambda4 = 4.741341;
    guess.setMultiplier(
            "lambda_cid3_p4", createVectorLinspace(Nguess, lambda4, lambda4));
    const double lambda5 = 0.691361;
    guess.setMultiplier(
            "lambda_cid3_p5", createVectorLinspace(Nguess, lambda5, lambda5));

    // Set derivatives (accelerations) to "random" values.
    const double rz_a = .4024742;
    guess.setDerivative("/jointset/joint/rz/accel",
            createVectorLinspace(Nguess, rz_a, rz_a));
    const double tx_a = 0.351;
    guess.setDerivative("/jointset/joint/tx/accel",
            createVectorLinspace(Nguess, tx_a, tx_a));
    const double ty_a = 0.601361;
    guess.setDerivative("/jointset/joint/ty/accel",
            createVectorLinspace(Nguess, ty_a, ty_a));

    solver.setGuess(guess);

    MocoSolution solution = study.solve().unseal();

    // Make sure the solution contains the multipliers and derivatives we
    // supplied, not any computed by Simbody.

    CHECK(solution.getMultiplier("lambda_cid3_p0").getElt(0, 0) ==
            Approx(lambda0));
    CHECK(solution.getMultiplier("lambda_cid3_p1").getElt(0, 0) ==
            Approx(lambda1));
    CHECK(solution.getMultiplier("lambda_cid3_p2").getElt(0, 0) ==
            Approx(lambda2));
    CHECK(solution.getMultiplier("lambda_cid3_p3").getElt(0, 0) ==
            Approx(lambda3));
    CHECK(solution.getMultiplier("lambda_cid3_p4").getElt(0, 0) ==
            Approx(lambda4));
    CHECK(solution.getMultiplier("lambda_cid3_p5").getElt(0, 0) ==
            Approx(lambda5));

    CHECK(solution.getDerivative("/jointset/joint/rz/accel").getElt(0, 0) ==
            Approx(rz_a));
    CHECK(solution.getDerivative("/jointset/joint/tx/accel").getElt(0, 0) ==
            Approx(tx_a));
    CHECK(solution.getDerivative("/jointset/joint/ty/accel").getElt(0, 0) ==
            Approx(ty_a));

    CHECK(testData->udot[0] == Approx(rz_a));
    CHECK(testData->udot[1] == Approx(tx_a));
    CHECK(testData->udot[2] == Approx(ty_a));

    // The constraint Jacobian is:
    // [1 0 0
    //  0 0 0
    //  0 0 0
    //  0 1 0
    //  0 0 1
    //  0 0 0].
    // That is, lambda0 exerts a moment in direction rz, lambda3 exerts
    // a force in direction tx, and lambda4 exerts a force in direction ty.
    auto state = model.initSystem();
    model.setStateVariableValue(state, "/jointset/joint/rz/value", rz);
    model.setStateVariableValue(state, "/jointset/joint/tx/value", tx);
    model.setStateVariableValue(state, "/jointset/joint/ty/value", ty);
    model.realizePosition(state);
    SimTK::Matrix G;
    model.getMatterSubsystem().calcG(state, G);
    CAPTURE(G);

    // Compute the joint reactions from the Lagrange multipliers (lambda's) and
    // compare them to joint reactions computed internally.
    double Mx = lambda1;
    double My = lambda2;
    // Mz - lambda0 = Izz * alpha_z
    double Mz = lambda0 + 1.0 * rz_a;
    double Rx = mass * tx_a + lambda3;
    double g = model.get_gravity().norm();
    double Ry = mass * ty_a + mass * g + lambda4;
    double Rz = lambda5;

    CHECK(Mx == Approx(testData->reaction[0][0]));
    CHECK(My == Approx(testData->reaction[0][1]));
    CHECK(Mz == Approx(testData->reaction[0][2]));
    CHECK(Rx == Approx(testData->reaction[1][0]));
    CHECK(Ry == Approx(testData->reaction[1][1]));
    CHECK(Rz == Approx(testData->reaction[1][2]));
}

TEST_CASE("Multipliers are correct", "[casadi]") {
    SECTION("Body welded to ground") {
        auto dynamics_mode =
                GENERATE(as<std::string>{}, "implicit", "explicit");
        auto kinematic_constraint_method =
                GENERATE(as<std::string>{}, "Posa2016", "Bordalba2023");

        Model model;
        const double mass = 1.3169;
        auto* body = new Body("body", mass, SimTK::Vec3(0), SimTK::Inertia(1));
        model.addBody(body);

        auto* joint = new PlanarJoint("joint", model.getGround(), *body);
        model.addJoint(joint);

        auto* constr = new PointConstraint(model.getGround(), Vec3(0),
                                           *body, Vec3(0));
        model.addConstraint(constr);
        model.finalizeConnections();

        MocoStudy study;
        auto& problem = study.updProblem();
        problem.setModelAsCopy(model);

        problem.setTimeBounds(0, 0.5);

        auto& solver = study.initCasADiSolver();
        solver.set_num_mesh_intervals(5);
        solver.set_multibody_dynamics_mode(dynamics_mode);
        solver.set_kinematic_constraint_method(kinematic_constraint_method);
        solver.set_transcription_scheme("hermite-simpson");
        solver.set_enforce_constraint_derivatives(true);

        MocoSolution solution = study.solve();

        // Constraints 0 through 2 are the locks for the 3 translational DOFs.
        const auto FX = solution.getMultiplier("lambda_cid3_p0");
        SimTK::Vector zero(FX);
        zero.setToZero();
        OpenSim_CHECK_MATRIX_TOL(FX, zero, 1e-5);
        const auto FY = solution.getMultiplier("lambda_cid3_p1");
        SimTK::Vector g(zero.size(), model.get_gravity()[1]);
        OpenSim_CHECK_MATRIX_TOL(FY, mass * g, 1e-5);
        const auto FZ = solution.getMultiplier("lambda_cid3_p2");
        OpenSim_CHECK_MATRIX_TOL(FZ, zero, 1e-5);
    }

    // This problem is a point mass constrained to the line 0 = x - y.
    // constraint Jacobian G is [1, -1].
    //      m xdd + G(0) * lambda = Fx  -> m xdd + lambda = Fx
    //      m ydd + G(1) * lambda = Fy  -> m ydd - lambda = Fy
    // Since xdd = ydd, we have:
    //      lambda = 0.5 * (Fx - Fy).
    // This test ensures that the multiplier has the correct value.
    SECTION("Planar point mass with CoordinateCouplerConstraint") {

        auto dynamics_mode =
                GENERATE(as<std::string>{}, "implicit", "explicit");
        auto kinematic_constraint_method =
                GENERATE(as<std::string>{}, "Posa2016", "Bordalba2023");

        Model model = ModelFactory::createPlanarPointMass();
        model.set_gravity(Vec3(0));
        CoordinateCouplerConstraint* constraint =
                new CoordinateCouplerConstraint();
        Array<std::string> names;
        names.append("tx");
        constraint->setIndependentCoordinateNames(names);
        constraint->setDependentCoordinateName("ty");
        LinearFunction func(1.0, 0.0);
        constraint->setFunction(func);
        model.addConstraint(constraint);

        model.finalizeConnections();

        MocoStudy study;
        auto& problem = study.updProblem();
        problem.setModelAsCopy(model);

        problem.setTimeBounds(0, 1);
        problem.setStateInfo("/jointset/tx/tx/value", {-5, 5}, 0, 3);
        problem.setStateInfo("/jointset/tx/tx/speed", {-5, 5}, 0, 0);
        problem.setControlInfo("/forceset/force_x", 0.5);

        problem.addGoal<MocoControlGoal>();

        auto& solver = study.initCasADiSolver();
        solver.set_num_mesh_intervals(10);
        solver.set_multibody_dynamics_mode(dynamics_mode);
        solver.set_transcription_scheme("hermite-simpson");
        solver.set_kinematic_constraint_method(kinematic_constraint_method);
        solver.set_enforce_constraint_derivatives(true);
        MocoSolution solution = study.solve();
        const auto Fx = solution.getControl("/forceset/force_x");
        const auto Fy = solution.getControl("/forceset/force_y");
        const auto lambda = solution.getMultiplier("lambda_cid2_p0");

        OpenSim_CHECK_MATRIX_TOL(lambda, 0.5 * (Fx - Fy), 1e-5);
    }
}

// Ensure that we correctly handle the combination of prescribed kinematics
// (PositionMotion) and kinematic constraints. This test is similar to the one
// above except that we prescribe motions for tx and ty.
TEST_CASE("Prescribed kinematics with kinematic constraints", "[casadi]") {
    Model model = ModelFactory::createPlanarPointMass();
    model.set_gravity(Vec3(0));
    CoordinateCouplerConstraint* constraint = new CoordinateCouplerConstraint();
    Array<std::string> names;
    names.append("tx");
    constraint->setIndependentCoordinateNames(names);
    constraint->setDependentCoordinateName("ty");
    LinearFunction func(1.0, 0.0);
    constraint->setFunction(func);
    model.addConstraint(constraint);

    auto* posmot = new PositionMotion();
    Sine function = Sine(1.0, 1.0, 0, 1.0);
    posmot->setPositionForCoordinate(model.getCoordinateSet().get(0), function);
    posmot->setPositionForCoordinate(model.getCoordinateSet().get(1), function);
    model.addComponent(posmot);

    model.finalizeConnections();

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(model);

    problem.setTimeBounds(0, 3);
    problem.setControlInfo("/forceset/force_x", 0.5);

    problem.addGoal<MocoControlGoal>();

    auto& solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(10);
    solver.set_multibody_dynamics_mode("implicit");
    solver.set_interpolate_control_mesh_interior_points(false);
    MocoSolution solution = study.solve();
    const auto Fx = solution.getControl("/forceset/force_x");
    const auto Fy = solution.getControl("/forceset/force_y");
    const auto lambda = solution.getMultiplier("lambda_cid2_p0");

    OpenSim_CHECK_MATRIX_TOL(lambda, 0.5 * (Fx - Fy), 1e-5);
}

TEMPLATE_TEST_CASE("MocoControlBoundConstraint", "",
        MocoCasADiSolver) {
    SECTION("Lower bound only") {
        MocoStudy study;
        auto& problem = study.updProblem();
        problem.setModelAsCopy(ModelFactory::createPendulum());
        problem.setTimeBounds(0, 1);
        problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0);
        problem.setStateInfo("/jointset/j0/q0/speed", {-10, 10}, 0);
        problem.setControlInfo("/tau0", {-5, 5});
        problem.addGoal<MocoControlGoal>();
        auto* constr = problem.addPathConstraint<MocoControlBoundConstraint>();
        const double lowerBound = 0.1318;
        constr->addControlPath("/tau0");
        constr->setLowerBound(Constant(lowerBound));

        study.initSolver<TestType>();
        MocoSolution solution = study.solve();
        SimTK::Vector expected(solution.getNumTimes());
        expected = lowerBound;
        OpenSim_CHECK_MATRIX_ABSTOL(
                solution.getControlsTrajectory(), expected, 1e-6);
    }

    SECTION("Upper bound only") {
        MocoStudy study;
        auto& problem = study.updProblem();
        problem.setModelAsCopy(ModelFactory::createPendulum());
        problem.setTimeBounds(0, {0.1, 10});
        problem.setStateInfo("/jointset/j0/q0/value", {0, 1}, 0, 0.53);
        problem.setStateInfo("/jointset/j0/q0/speed", {-10, 10}, 0, 0);
        problem.setControlInfo("/tau0", {-20, 20});
        problem.addGoal<MocoFinalTimeGoal>();
        auto* constr = problem.addPathConstraint<MocoControlBoundConstraint>();
        constr->addControlPath("/tau0");
        const double upperBound = 11.236;
        constr->setUpperBound(Constant(upperBound));

        study.initSolver<TestType>();
        MocoSolution solution = study.solve();
        SimTK::Vector expected(solution.getNumTimes());
        expected = upperBound;
        CHECK(SimTK::max(solution.getControlsTrajectory())[0] ==
                Approx(upperBound).margin(1e-6));
        CHECK(SimTK::min(solution.getControlsTrajectory())[0] ==
                Approx(-20).margin(1e-6));
    }

    SECTION("Upper and lower bounds are the same") {
        MocoStudy study;
        auto& problem = study.updProblem();
        problem.setModelAsCopy(ModelFactory::createPendulum());
        problem.setTimeBounds(0, 1);
        problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0);
        problem.setStateInfo("/jointset/j0/q0/speed", {-10, 10}, 0);
        problem.setControlInfo("/tau0", {-5, 5});
        problem.addGoal<MocoControlGoal>();
        PiecewiseLinearFunction violateLower;
        violateLower.addPoint(0, 0);
        violateLower.addPoint(0.2, 0.5316);
        violateLower.addPoint(0.7, -0.3137);
        violateLower.addPoint(1, .0319);
        auto* constr = problem.addPathConstraint<MocoControlBoundConstraint>();
        constr->addControlPath("/tau0");
        constr->setLowerBound(violateLower);
        constr->setEqualityWithLower(true);
        study.initSolver<TestType>();
        MocoSolution solution = study.solve();
        SimTK::Vector expectedV(solution.getNumTimes());
        for (int itime = 0; itime < expectedV.size(); ++itime) {
            SimTK::Vector arg(1);
            arg[0] = solution.getTime()[itime];
            expectedV[itime] = violateLower.calcValue(arg);
        }
        MocoTrajectory expected = solution;
        expected.setControl("/tau0", expectedV);

        CHECK(solution.compareContinuousVariablesRMS(
                      expected, {{"controls", {}}}) < 1e-3);
    }

    SECTION("Time range of bounds function is too small.") {
        MocoStudy study;
        auto& problem = study.updProblem();
        problem.setModelAsCopy(ModelFactory::createPendulum());
        problem.setTimeBounds({-31, 0}, {1, 50});
        problem.addGoal<MocoControlGoal>();
        GCVSpline violateLower;
        violateLower.setDegree(5);
        violateLower.addPoint(-30.9999, 0);
        violateLower.addPoint(0, 0);
        violateLower.addPoint(0.5, 0);
        violateLower.addPoint(0.7, 0);
        violateLower.addPoint(0.8, 0);
        violateLower.addPoint(0.9, 0);
        violateLower.addPoint(50, 0.319);
        auto* constr = problem.addPathConstraint<MocoControlBoundConstraint>();
        constr->addControlPath("/tau0");
        constr->setLowerBound(violateLower);
        CHECK_THROWS_WITH(study.solve(),
                ContainsSubstring("must be less than or equal to the minimum"));
        constr->clearLowerBound();
        GCVSpline violateUpper;
        violateUpper.setDegree(5);
        violateUpper.addPoint(-31, 0);
        violateUpper.addPoint(0, 0);
        violateUpper.addPoint(0.5, 0);
        violateUpper.addPoint(0.7, 0);
        violateUpper.addPoint(0.8, 0);
        violateUpper.addPoint(0.9, 0);
        violateUpper.addPoint(49.99999, .0319);
        constr->setUpperBound(violateUpper);
        CHECK_THROWS_WITH(study.solve(),
                ContainsSubstring(
                    "must be greater than or equal to the maximum"));
    }

    SECTION("Can omit both bounds.") {
        MocoStudy study;
        auto& problem = study.updProblem();
        problem.setModelAsCopy(ModelFactory::createPendulum());
        problem.setTimeBounds(0, 1);
        problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0);
        problem.setStateInfo("/jointset/j0/q0/speed", {-10, 10}, 0);
        problem.setControlInfo("/tau0", {-5, 5});
        problem.addGoal<MocoControlGoal>();
        auto* constr = problem.addPathConstraint<MocoControlBoundConstraint>();
        study.initSolver<TestType>();
        study.solve();
        constr->addControlPath("/tau0");
        study.solve();
    }
}

TEMPLATE_TEST_CASE("MocoOutputBoundConstraint", "",
        MocoCasADiSolver) {
    SECTION("Two outputs with equality bound") {
        MocoSolution solutionControl;
        MocoStudy study;
        auto& problem = study.updProblem();
        auto model = createDoubleSlidingMassModel();
        model->initSystem();

        problem.setModelAsCopy(*model);
        problem.setTimeBounds(0, 6.283);

        problem.setStateInfo("/slider/position/value", MocoBounds(-5, 5),
            MocoInitialBounds(-1, 1));
        problem.setStateInfo("/slider/position/speed", {-1, 1});
        problem.setStateInfo("/slider2/position/value", MocoBounds(-5, 5),
            MocoInitialBounds(-1, 1));
        problem.setStateInfo("/slider2/position/speed", {-1, 1});

        auto* constr = problem.addPathConstraint<MocoOutputBoundConstraint>();
        Sine lower;
        constr->setLowerBound(lower);
        constr->setEqualityWithLower(true);
        constr->setOutputIndex(0);
        constr->setOutputPath("/body|position");
        constr->setSecondOutputPath("/body2|position");
        constr->setOperation("subtraction");

        auto& solver = study.template initSolver<TestType>();
        solver.set_num_mesh_intervals(30);
        MocoSolution solution = study.solve();

        auto solutionPosition1 = solution.getState("/slider/position/value");
        auto solutionPosition2 = solution.getState("/slider2/position/value");
        auto times = solution.getTime();
        SimTK::Vector time(1);
        for (int i = 0; i < solution.getNumTimes(); ++i) {
            double diff = (static_cast<SimTK::Vec3>(solutionPosition1[i])[0]
                - static_cast<SimTK::Vec3>(solutionPosition2[i])[0]);
            time[0] = times[i];
            REQUIRE_THAT(diff, Catch::Matchers::WithinAbs(
                               lower.calcValue(time), 1e-3));
        }
    }

    SECTION("One output with upper bound") {
        MocoSolution solutionControl;
        MocoStudy study;
        auto& problem = study.updProblem();
        auto model = ModelFactory::createSlidingPointMass();
        model.initSystem();

        problem.setModelAsCopy(model);
        problem.setTimeBounds(0, 3);

        problem.setStateInfo("/slider/position/value", MocoBounds(-3, 3));
        problem.setStateInfo("/slider/position/speed", {-3, 3});

        auto* constr = problem.addPathConstraint<MocoOutputBoundConstraint>();
        PiecewiseLinearFunction upper;
        upper.addPoint(0, 0);
        upper.addPoint(1, -1);
        upper.addPoint(2, -0.4);
        upper.addPoint(3, 1);
        constr->setUpperBound(upper);
        constr->setOutputIndex(0);
        constr->setOutputPath("/body|position");

        // want to minimize position magnitude, but can't reach 0 due to constraint
        auto* goal = problem.addGoal<MocoOutputGoal>();
        goal->setOutputPath("/body|position");
        goal->setExponent(2);

        auto* effort = problem.addGoal<MocoControlGoal>();
        effort->setWeight(0.1);
        effort->setName("effort");

        auto& solver = study.template initSolver<TestType>();
        solver.set_num_mesh_intervals(30);
        MocoSolution solution = study.solve();

        auto solutionPosition = solution.getState("/slider/position/value");
        auto times = solution.getTime();
        SimTK::Vector time(1);
        for (int i = 0; i < solution.getNumTimes(); ++i) {
            double position = static_cast<SimTK::Vec3>(solutionPosition[i])[0];
            time[0] = times[i];
            double bound = upper.calcValue(time);
            CHECK(Catch::Approx(position).margin(1e-4) <= bound);
        }
    }

    SECTION("Time range of bounds function is too small.") {
        MocoStudy study;
        auto& problem = study.updProblem();
        problem.setModelAsCopy(ModelFactory::createPendulum());
        problem.setTimeBounds({-31, 0}, {1, 50});
        problem.addGoal<MocoControlGoal>();
        GCVSpline violateLower;
        violateLower.setDegree(5);
        violateLower.addPoint(-30.9999, 0);
        violateLower.addPoint(0, 0);
        violateLower.addPoint(0.5, 0);
        violateLower.addPoint(0.7, 0);
        violateLower.addPoint(0.8, 0);
        violateLower.addPoint(0.9, 0);
        violateLower.addPoint(50, 0.319);
        auto* constr = problem.addPathConstraint<MocoOutputBoundConstraint>();
        constr->setOutputPath("|kinetic_energy");
        constr->setLowerBound(violateLower);
        CHECK_THROWS_WITH(study.solve(),
                ContainsSubstring("must be less than or equal to the minimum"));
        constr->clearLowerBound();
        GCVSpline violateUpper;
        violateUpper.setDegree(5);
        violateUpper.addPoint(-31, 0);
        violateUpper.addPoint(0, 0);
        violateUpper.addPoint(0.5, 0);
        violateUpper.addPoint(0.7, 0);
        violateUpper.addPoint(0.8, 0);
        violateUpper.addPoint(0.9, 0);
        violateUpper.addPoint(49.99999, .0319);
        constr->setUpperBound(violateUpper);
        CHECK_THROWS_WITH(study.solve(),
                ContainsSubstring(
                    "must be greater than or equal to the maximum"));
    }

    SECTION("Can omit both bounds.") {
        MocoStudy study;
        auto& problem = study.updProblem();
        problem.setModelAsCopy(ModelFactory::createPendulum());
        problem.setTimeBounds(0, 1);
        problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0);
        problem.setStateInfo("/jointset/j0/q0/speed", {-10, 10}, 0);
        problem.setControlInfo("/tau0", {-5, 5});
        problem.addGoal<MocoControlGoal>();
        auto* constr = problem.addPathConstraint<MocoOutputBoundConstraint>();
        constr->setOutputPath("|kinetic_energy");
        study.initSolver<TestType>();
        study.solve();
    }
}

TEMPLATE_TEST_CASE("MocoStateBoundConstraint", "",
        MocoCasADiSolver) {
    SECTION("Upper bounded speed") {
        MocoSolution solutionControl;
        MocoStudy study;
        auto& problem = study.updProblem();
        problem.setModelAsCopy(ModelFactory::createSlidingPointMass());
        problem.setTimeBounds(0, 3);
        problem.setStateInfo("/slider/position/speed", {-10, 10}, 0);
        auto* effort = problem.addGoal<MocoControlGoal>();
        effort->setName("effort");
        effort->setWeight(0.001);
        // decreasing speed constraint
        auto* constr = problem.addPathConstraint<MocoStateBoundConstraint>();
        constr->addStatePath("/slider/position/speed");
        PiecewiseLinearFunction upperBound;
        upperBound.addPoint(0, 2);
        upperBound.addPoint(3, -3);
        constr->setUpperBound(upperBound);
        study.initSolver<TestType>();
        MocoSolution solution = study.solve();
        // ensure that at every state, the speed is under the bound
        auto solutionSpeed = solution.getState("/slider/position/speed");
        auto times = solution.getTime();
        SimTK::Vector time(1);
        for (int i = 0; i < solution.getNumTimes(); ++i) {
            double speed = solutionSpeed[i];
            time[0] = times[i];
            double max = upperBound.calcValue(time);
            CHECK(Catch::Approx(speed).margin(1e-8) <= max);
        }
    }

    SECTION("Equality bounded speed") {
        MocoSolution solutionControl;
        MocoStudy study;
        auto& problem = study.updProblem();
        problem.setModelAsCopy(ModelFactory::createSlidingPointMass());
        problem.setTimeBounds(0, 10);
        problem.setStateInfo("/slider/position/speed", {-10, 10}, 0);
        // add constraint of changing speed
        auto* constr = problem.addPathConstraint<MocoStateBoundConstraint>();
        constr->addStatePath("/slider/position/speed");
        constr->setEqualityWithLower(true);
        Sine lowerBound;
        constr->setLowerBound(lowerBound);
        // can't have upper bound when set to equal lower bound
        constr->setUpperBound(Constant(1));
        CHECK_THROWS(study.initSolver<TestType>());
        // undo upper bound and solve
        constr->clearUpperBound();
        study.initSolver<TestType>();
        MocoSolution solution = study.solve();
        // check that the speed is always close to the lower bound
        auto solutionSpeed = solution.getState("/slider/position/speed");
        auto times = solution.getTime();
        SimTK::Vector time(1);
        for (int i = 0; i < solution.getNumTimes(); ++i) {
            double speed = solutionSpeed[i];
            time[0] = times[i];
            double max = lowerBound.calcValue(time);
            REQUIRE_THAT(speed, Catch::Matchers::WithinAbs(max, 1e-4));
        }
    }

    SECTION("Double bounded speed") {
        MocoSolution solutionControl;
        MocoStudy study;
        auto& problem = study.updProblem();
        problem.setModelAsCopy(ModelFactory::createSlidingPointMass());
        problem.setTimeBounds(0, 4);
        problem.setStateInfo("/slider/position/speed", {-10, 10});
        // add speed constraint with changing upper and lower bounds
        auto* constr = problem.addPathConstraint<MocoStateBoundConstraint>();
        constr->addStatePath("/slider/position/speed");
        PiecewiseLinearFunction lowerBound;
        lowerBound.addPoint(0, -2);
        lowerBound.addPoint(2, 1);
        lowerBound.addPoint(4, -3);
        constr->setLowerBound(lowerBound);
        PiecewiseLinearFunction upperBound;
        upperBound.addPoint(0, -1);
        upperBound.addPoint(2, 4);
        upperBound.addPoint(4, 0);
        constr->setUpperBound(upperBound);
        study.initSolver<TestType>();
        MocoSolution solution = study.solve();
        // check that the speed is between the bounds
        auto solutionSpeed = solution.getState("/slider/position/speed");
        auto times = solution.getTime();
        SimTK::Vector time(1);
        for (int i = 0; i < solution.getNumTimes(); ++i) {
            double speed = solutionSpeed[i];
            time[0] = times[i];
            double max = upperBound.calcValue(time);
            CHECK(Catch::Approx(speed).margin(1e-8) <= max);
            double min = lowerBound.calcValue(time);
            CHECK(Catch::Approx(speed).margin(1e-8) >= min);
        }
    }

    SECTION("Time range of bounds function is too small.") {
        MocoStudy study;
        auto& problem = study.updProblem();
        problem.setModelAsCopy(ModelFactory::createPendulum());
        problem.setTimeBounds({-31, 0}, {1, 50});
        problem.addGoal<MocoControlGoal>();
        GCVSpline violateLower;
        violateLower.setDegree(5);
        violateLower.addPoint(-30.9999, 0);
        violateLower.addPoint(0, 0);
        violateLower.addPoint(0.5, 0);
        violateLower.addPoint(0.7, 0);
        violateLower.addPoint(0.8, 0);
        violateLower.addPoint(0.9, 0);
        violateLower.addPoint(50, 0.319);
        auto* constr = problem.addPathConstraint<MocoStateBoundConstraint>();
        constr->addStatePath("/jointset/j0/q0/value");
        constr->setLowerBound(violateLower);
        CHECK_THROWS_WITH(study.solve(),
                ContainsSubstring("must be less than or equal to the minimum"));

        constr->clearLowerBound();
        GCVSpline violateUpper;
        violateUpper.setDegree(5);
        violateUpper.addPoint(-31, 0);
        violateUpper.addPoint(0, 0);
        violateUpper.addPoint(0.5, 0);
        violateUpper.addPoint(0.7, 0);
        violateUpper.addPoint(0.8, 0);
        violateUpper.addPoint(0.9, 0);
        violateUpper.addPoint(49.99999, .0319);
        constr->setUpperBound(violateUpper);
        CHECK_THROWS_WITH(study.solve(),
                ContainsSubstring(
                    "must be greater than or equal to the maximum"));
    }

    SECTION("Can omit both bounds.") {
        MocoStudy study;
        auto& problem = study.updProblem();
        problem.setModelAsCopy(ModelFactory::createPendulum());
        problem.setTimeBounds(0, 1);
        problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0);
        problem.setStateInfo("/jointset/j0/q0/speed", {-10, 10}, 0);
        problem.setControlInfo("/tau0", {-5, 5});
        problem.addGoal<MocoControlGoal>();
        auto* constr = problem.addPathConstraint<MocoStateBoundConstraint>();
        study.initSolver<TestType>();
        study.solve();
        constr->addStatePath("/jointset/j0/q0/speed");
        study.solve();
    }
}

TEMPLATE_TEST_CASE("MocoFrameDistanceConstraint", "",
        MocoCasADiSolver) {
    using SimTK::Pi;

    // Create a 3D pendulum model with a single body and a marker at the
    // end-effector.
    Model model;
    auto* body = new Body("body", 1, Vec3(0), SimTK::Inertia(1));
    model.addBody(body);
    auto* joint = new GimbalJoint("gimbal", model.getGround(), Vec3(0, 1, 0),
            Vec3(0, 0, -Pi/2), *body, Vec3(-1, 0, 0), Vec3(0));
    auto& qx = joint->updCoordinate(GimbalJoint::Coord::Rotation1X);
    qx.setName("qx");
    auto& qy = joint->updCoordinate(GimbalJoint::Coord::Rotation2Y);
    qy.setName("qy");
    auto& qz = joint->updCoordinate(GimbalJoint::Coord::Rotation3Z);
    qz.setName("qz");
    model.addJoint(joint);
    auto* marker = new Marker("marker", *body, Vec3(0));
    model.addMarker(marker);
    Ellipsoid bodyGeometry(0.5, 0.1, 0.1);
    bodyGeometry.setColor(SimTK::Gray);
    PhysicalOffsetFrame* body_center = new PhysicalOffsetFrame(
            "body_center", *body, SimTK::Transform(Vec3(-0.5, 0, 0)));
    body->addComponent(body_center);
    body_center->attachGeometry(bodyGeometry.clone());
    model.finalizeConnections();

    // Optimize the trajectory of the 3D pendulum to meet a final marker
    // position goal while obeying a minimum distance between the ground origin
    // and the marker (coincident with body frame origin).
    MocoStudy study;
    auto& problem = study.updProblem();
    ModelProcessor modelProcessor(model);
    modelProcessor.append(ModOpAddReserves(50));
    problem.setModelProcessor(modelProcessor);
    problem.setTimeBounds(0, 0.5);
    problem.setStateInfo("/jointset/gimbal/qx/value", {-Pi/3, Pi/3}, 0);
    problem.setStateInfo(
            "/jointset/gimbal/qy/value", {-Pi/3, Pi/3}, Pi/4, -Pi/4);
    problem.setStateInfo("/jointset/gimbal/qz/value", {-Pi/3, Pi/3}, 0);
    problem.setStateInfo("/jointset/gimbal/qx/speed", {-10, 10}, 0, 0);
    problem.setStateInfo("/jointset/gimbal/qy/speed", {-10, 10}, 0, 0);
    problem.setStateInfo("/jointset/gimbal/qz/speed", {-10, 10}, 0, 0);

    auto* distanceConstraint =
        problem.addPathConstraint<MocoFrameDistanceConstraint>();
    const double distance = 0.1;
    distanceConstraint->addFramePair({"/ground", "/bodyset/body", distance,
        SimTK::Infinity});

    auto* finalMarkerGoal =
            problem.addGoal<MocoMarkerFinalGoal>("final_marker");
    finalMarkerGoal->setPointName("/markerset/marker");
    finalMarkerGoal->setReferenceLocation(Vec3(0, 0.292893, 0.707107));

    study.initSolver<TestType>();
    MocoSolution solution = study.solve();
    //study.visualize(solution);

    TimeSeriesTableVec3 positionTable =
            analyzeMocoTrajectory<SimTK::Vec3>(modelProcessor.process(),
                    solution, {"/markerset/marker\\|location"});
    SimTK::Vec3 position;
    for (int irow = 0; irow < (int)positionTable.getNumRows(); ++irow) {
        position = positionTable.getRowAtIndex(irow)[0];
        // The margin is looser than the constraint tolerance because the
        // constraint is on the square of the distance.
        CHECK(Approx(position.norm()).margin(1e-2) >= distance);
    }
}

TEMPLATE_TEST_CASE("Multiple MocoPathConstraints", "", MocoCasADiSolver) {
    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(ModelFactory::createDoublePendulum());
    problem.setTimeBounds(0, 1);
    problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0);
    problem.setStateInfo("/jointset/j0/q0/speed", {-10, 10}, 0);
    problem.setStateInfo("/jointset/j1/q1/value", {-10, 10}, 0);
    problem.setStateInfo("/jointset/j1/q1/speed", {-10, 10}, 0);
    problem.setControlInfo("/tau0", {-5, 5});
    problem.setControlInfo("/tau0", {-5, 5});
    problem.addGoal<MocoControlGoal>();
    auto* controlConstraint = problem
            .addPathConstraint<MocoControlBoundConstraint>();
    controlConstraint->setName("control_constraint");
    controlConstraint->addControlPath("/tau0");
    controlConstraint->setUpperBound(Constant(1.0));
    auto* distanceConstraint =
            problem.addPathConstraint<MocoFrameDistanceConstraint>();
    distanceConstraint->setName("distance_constraint");
    distanceConstraint->addFramePair({"/ground", "/bodyset/b0", 1.0,
                                      SimTK::Infinity});
    distanceConstraint->addFramePair({"/ground", "/bodyset/b1", 1.0,
                                      SimTK::Infinity});
    study.initSolver<TestType>();
    study.solve();
}


class ConstantSpeedConstraint : public Constraint {
OpenSim_DECLARE_CONCRETE_OBJECT(ConstantSpeedConstraint, Constraint);

public:
    OpenSim_DECLARE_SOCKET(body, PhysicalFrame,
        "The body participating in this constraint.");

private:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        const PhysicalFrame& f = getConnectee<PhysicalFrame>("body");
        SimTK::MobilizedBody b = f.getMobilizedBody();
        SimTK::Constraint::ConstantSpeed simtkConstantSpeed(b,
            SimTK::MobilizerUIndex(0), SimTK::Real(2.34));
        assignConstraintIndex(simtkConstantSpeed.getConstraintIndex());
    }
};


class ConstantAccelerationConstraint : public Constraint {
OpenSim_DECLARE_CONCRETE_OBJECT(ConstantAccelerationConstraint, Constraint);

public:
    OpenSim_DECLARE_SOCKET(body, PhysicalFrame,
        "The body participating in this constraint.");

private:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        const PhysicalFrame& f = getConnectee<PhysicalFrame>("body");
        SimTK::MobilizedBody b = f.getMobilizedBody();
        SimTK::Constraint::ConstantAcceleration simtkConstantAcceleration(b,
            SimTK::MobilizerUIndex(0), SimTK::Real(1.23));
        assignConstraintIndex(simtkConstantAcceleration.getConstraintIndex());
    }
};

MocoStudy createSlidingMassMocoStudy(const Model& model, 
        const std::string& scheme) {
    MocoStudy study;
    study.setName("sliding_mass");
    MocoProblem& mp = study.updProblem();
    mp.setModel(std::make_unique<Model>(model));
    mp.setTimeBounds(0, {0, 10});
    mp.setStateInfo("/slider/position/value",{0, 1}, 0, 1);
    mp.setStateInfo("/slider/position/speed", {-100, 100});
    mp.addGoal<MocoControlGoal>();

    auto& ms = study.initCasADiSolver();
    ms.set_num_mesh_intervals(50);
    ms.set_transcription_scheme(scheme);
    ms.set_kinematic_constraint_method("Bordalba2023");

    return study;
}

TEST_CASE("ConstantSpeedConstraint") {
    std::string scheme = GENERATE(as<std::string>{},
            "trapezoidal", "hermite-simpson", "legendre-gauss-3", 
            "legendre-gauss-radau-3");

    Model model = ModelFactory::createSlidingPointMass();
    auto* constraint = new ConstantSpeedConstraint();
    constraint->setName("constant_speed");
    constraint->connectSocket_body(model.getComponent<Body>("body"));
    model.addConstraint(constraint);
    model.initSystem();

    MocoStudy study = createSlidingMassMocoStudy(model, scheme);
    MocoSolution solution = study.solve();

    const auto& speed = solution.getState("/slider/position/speed");
    for (int itime = 0; itime < solution.getNumTimes(); ++itime) {
        CHECK(speed[itime] == Approx(2.34).margin(1e-9));
    }
}

TEST_CASE("ConstantAccelerationConstraint") {
    std::string scheme = GENERATE(as<std::string>{},
            "trapezoidal", "hermite-simpson", "legendre-gauss-3", 
            "legendre-gauss-radau-3");

    Model model = ModelFactory::createSlidingPointMass();
    auto* constraint = new ConstantAccelerationConstraint();
    constraint->setName("constant_acceleration");
    constraint->connectSocket_body(model.getComponent<Body>("body"));
    model.addConstraint(constraint);
    model.initSystem();

    MocoStudy study = createSlidingMassMocoStudy(model, scheme);
    MocoSolution solution = study.solve();

    const auto& states = solution.exportToStatesTrajectory(model);
    for (const auto& state : states) {
        model.realizeAcceleration(state);
        CHECK(state.getUDot()[0] == Approx(1.23).margin(1e-9));
    }
}

