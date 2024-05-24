/* -------------------------------------------------------------------------- *
 *                         OpenSim:  testWrapping.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/OpenSim.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Simulation/Model/FunctionBasedPath.h>
#include <OpenSim/Common/CommonUtilities.h>
#include <SimTKcommon.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;
using namespace SimTK;
using namespace std;
using OpenSim::TimeSeriesTable;
using Catch::Matchers::WithinAbs;

// HELPER FUNCTIONS
namespace {
    // Simulate a model from an initial time to a final time given an initial
    // state.
    void simulate(Model& model, State& state, double initialTime, 
            double finalTime)
    {
        // Create the Manager for the simulation.
        const double accuracy = 1.0e-4;
        Manager manager(model);
        manager.setIntegratorAccuracy(accuracy);
    
        // Integrate from initial time to final time.
        state.setTime(initialTime);
        cout << "\nIntegrating from " << initialTime << " to " 
             << finalTime << endl;
    
        const double start = SimTK::realTime();
        manager.initialize(state);
        manager.integrate(finalTime);
        cout << "Simulation time = " << SimTK::realTime() - start
             << " seconds (wallclock time)\n"
             << endl;
    
        auto& integrator = manager.getIntegrator();
        cout << "Integrator iterations = " 
             << integrator.getNumStepsTaken() << endl;
    
        // Save the simulation results.
        Storage states(manager.getStateStorage());
        states.print(model.getName() + "_states.sto");
        model.updSimbodyEngine().convertRadiansToDegrees(states);
        states.setWriteSIMMHeader(true);
        states.print(model.getName() + "_states_degrees.mot");
    }

    // Simulate a model with muscles at a constant activation to a given final
    // time.
    void simulateModelWithMuscles(
            const string& modelFile, double finalTime, double activation = 0.5) 
    {
        // Create a new OpenSim model.
        Model model(modelFile);
        
        // Create a PrescribedController that simply applies a function of the
        // force.
        PrescribedController actuController;
        const auto& actuatorsSet = model.getActuators();
        actuController.setActuators(actuatorsSet);
        const auto& socket = actuController.getSocket<Actuator>("actuators");
        for (int i = 0; i < (int)socket.getNumConnectees(); ++i) {
            actuController.prescribeControlForActuator(
                    actuatorsSet.get(i).getAbsolutePathString(),
                    Constant(activation));
        }
    
        // Add the controller to the model. We need to call disownAllComponents
        // because PrescribedController is on the stack.
        model.addController(&actuController);
        model.disownAllComponents();
        model.finalizeFromProperties();
        model.printBasicInfo();
    
        // Initialize the system and get the state representing the state system.
        SimTK::State& state = model.initSystem();
    
        // Apply the activations to the muscles and equilibrate.
        const Set<Muscle>& muscles = model.getMuscles();
        for (int i = 0; i < muscles.getSize(); i++) {
            muscles[i].setActivation(state, activation);
        }
        model.equilibrateMuscles(state);
    
        // Simulate.
        simulate(model, state, 0.0, finalTime);
    }
    
    // Given a TimeSeriesTable of coordinate values, create a TimeSeriesTable of
    // muscle lengths.
    TimeSeriesTable createMuscleLengthsTable(const OpenSim::Model& model,
            const TimeSeriesTable& coordinates) 
    {
        auto statesTraj = OpenSim::StatesTrajectory::createFromStatesTable(
                model, coordinates, true, true, true);
        TimeSeriesTable lengths;
        const auto& muscleSet = model.getMuscles();
        for (const auto& state : statesTraj) {
            model.realizePosition(state);
            SimTK::RowVector lengthsRow(muscleSet.getSize());
            for (int imuscle = 0; imuscle < muscleSet.getSize(); ++imuscle) {
                const auto& muscle = muscleSet.get(imuscle);
                const auto& geometryPath = muscle.getGeometryPath();
                lengthsRow[imuscle] = geometryPath.getLength(state);
            }
            lengths.appendRow(state.getTime(), lengthsRow);
        }
        std::vector<std::string> labels;
        for (int imuscle = 0; imuscle < muscleSet.getSize(); ++imuscle) {
            const auto& muscle = muscleSet.get(imuscle);
            const auto& path = muscle.getAbsolutePathString();
            labels.push_back(path + "|length");
        }
        lengths.setColumnLabels(labels);
        
        return lengths;
    }
}

TEST_CASE("testWrapCylinder") {
    const double r = 0.25;
    const double off = sqrt(2)*r-0.05;
    Model model;
    model.setName("testWrapCylinder");

    auto& ground = model.updGround();
    auto body = new OpenSim::Body("body", 1, Vec3(0), Inertia(0.1, 0.1, 0.01));
    model.addComponent(body);

    auto bodyOffset = new PhysicalOffsetFrame(
            "bToj", *body, Transform(Vec3(-off, 0, 0)));
    model.addComponent(bodyOffset);
    
    auto joint = new PinJoint("pin", ground, *bodyOffset);
    model.addComponent(joint);

    WrapCylinder* pulley1 = new WrapCylinder();
    pulley1->setName("pulley1");
    pulley1->set_radius(r);
    pulley1->set_length(0.05);

    // Add the wrap object to the body, which takes ownership of it
    ground.addWrapObject(pulley1);

    // One spring has wrap cylinder with respect to ground origin
    PathSpring* spring1 =
        new PathSpring("spring1", 1.0, 0.1, 0.01);
    spring1->updGeometryPath().
        appendNewPathPoint("origin", ground, Vec3(-off, 0, 0));
    spring1->updGeometryPath().
        appendNewPathPoint("insert", *body, Vec3(0));
    spring1->updGeometryPath().addPathWrap(*pulley1);

    model.addComponent(spring1);

    WrapCylinder* pulley2 = new WrapCylinder();
    pulley2->setName("pulley2");
    pulley2->set_radius(r);
    pulley2->set_length(0.05);

    // Add the wrap object to the body, which takes ownership of it
    bodyOffset->addWrapObject(pulley2);

    // Second spring has wrap cylinder with respect to bodyOffse origin
    PathSpring* spring2 =
        new PathSpring("spring2", 1.0, 0.1, 0.01);
    spring2->updGeometryPath().
        appendNewPathPoint("origin", ground, Vec3(-off, 0, 0));
    spring2->updGeometryPath().
        appendNewPathPoint("insert", *body, Vec3(0));
    spring2->updGeometryPath().addPathWrap(*pulley2);
    spring2->updGeometryPath().setDefaultColor(Vec3(0, 0.8, 0));

    model.addComponent(spring2);
    
    SimTK::State& s = model.initSystem();
    auto& coord = joint->updCoordinate();

    int nsteps = 10;
    for (int i = 0; i < nsteps; ++i) {
        
        coord.setValue(s, i*SimTK::Pi/(2*nsteps));
        model.realizeVelocity(s);
        
        double ma1 = spring1->computeMomentArm(s, coord);
        double ma2 = spring2->computeMomentArm(s, coord);

        CHECK_THAT(r - ma1, WithinAbs(0.0, SimTK::Eps));
        CHECK_THAT(r - ma2, WithinAbs(0.0, SimTK::Eps));

        double len1 = spring1->getLength(s);
        double len2 = spring2->getLength(s);

        CHECK_THAT(len1 - len2, WithinAbs(0.0, SimTK::Eps));
    }
}

TEST_CASE("testShoulderWrapping") {
    // Test the performance of multiple paths with wrapping in the 
    // upper-extremity.
    simulateModelWithMuscles("TestShoulderWrapping.osim", 0.1, 0.5);
}

TEST_CASE("testWrapObjectUpdateFromXMLNode30515") {
    // In XMLDocument version 30515, we converted VisibleObject, color and
    // display_preference properties to Appearance properties.
    XMLDocument doc("testWrapObject_updateFromXMLNode30515.osim");

    // Make sure this test is not weakened by the model in the repository being
    // updated.
    SimTK_TEST(doc.getDocumentVersion() == 20302);
    Model model("testWrapObject_updateFromXMLNode30515.osim");
    model.print("testWrapObject_updateFromXMLNode30515_updated.osim");
    const auto& wrapObjSet = model.getGround().getWrapObjectSet();

    // WrapSphere has:
    //   display_preference = 1
    //   color = default
    //   VisibleObject display_preference = 0
    {
        const auto& sphere = wrapObjSet.get("wrapsphere");
        SimTK_TEST(!sphere.get_Appearance().get_visible());
        SimTK_TEST_EQ(sphere.get_Appearance().get_color(), SimTK::Cyan);
        SimTK_TEST(sphere.get_Appearance().get_representation() == 
                VisualRepresentation::DrawPoints /* == 1 */);
    }

    // WrapCylinder has:
    //   display_preference = 0
    //   color = default
    //   VisibleObject display_preference = 4 
    {
        const auto& cyl = wrapObjSet.get("wrapcylinder");
        // The outer display_preference overrides the inner one.
        SimTK_TEST(!cyl.get_Appearance().get_visible());
        SimTK_TEST_EQ(cyl.get_Appearance().get_color(), SimTK::Cyan);
        SimTK_TEST(cyl.get_Appearance().get_representation() == 
                VisualRepresentation::DrawSurface /* == 3 */);
    }

    // WrapEllipsoid has:
    //   display_preference = 2
    //   color = 1 0.5 0
    //   VisibleObject display_preference = 3
    {
        const auto& ellipsoid = wrapObjSet.get("wrapellipsoid");
        SimTK_TEST(ellipsoid.get_Appearance().get_visible());
        SimTK_TEST_EQ(ellipsoid.get_Appearance().get_color(), Vec3(1, 0.5, 0));
        // Outer display_preference overrides inner one.
        SimTK_TEST(ellipsoid.get_Appearance().get_representation() == 
                VisualRepresentation::DrawWireframe /* == 2 */);
    }
}

TEST_CASE("testWrapObjectScaleWithNoFrameDoesNotSegfault") {
    // reproduction for #3465
    //
    // effectively, if code tries to use a `WrapObject` outside
    // of a `PhysicalFrame` then the code in this test will segfault
    // without the fix because `WrapObject` will contain nullptrs
    // that are usually "fixed" by the parent `PhysicalFrame`
    //
    // the "proper" fix for this is to (e.g.) use the socket API but
    // this was avoided in #3465, which just focuses on downgrading
    // the segfault

    try {
        OpenSim::Model m;
        m.addComponent(new OpenSim::WrapCylinder{});
        m.buildSystem();
        SimTK::State& state = m.initializeState();
        m.scale(state, OpenSim::ScaleSet{}, true);
    } catch (const OpenSim::Exception&) {
        // the fix in #3465 only ensures no runtime segfaults may
        // occur - it does not guarantee that `WrapObject`s are
        // usable outside of their usual usage (i.e. as children
        // of `PhysicalFrame`s)
    }
}

TEST_CASE("testMuscleLengths") {
    
    SECTION("Rajagopal2016, 18 muscles") {
        Model model("subject_walk_armless_18musc.osim");
        model.initSystem();
    
        TableProcessor tableProcessor =
                TableProcessor("subject_walk_armless_coordinates.mot") |
                TabOpUseAbsoluteStateNames();
        TimeSeriesTable coordinates =
                tableProcessor.processAndConvertToRadians(model);
        
        TimeSeriesTable lengths = createMuscleLengthsTable(model, coordinates);

        TimeSeriesTable stdLengths(
                "std_testMuscleLengths_subject_walk_armless.sto");
        CHECK(SimTK::Test::numericallyEqual(
                lengths.getMatrix(), stdLengths.getMatrix(),
                static_cast<int>(lengths.getNumColumns()), 1e-6));
    }
    
    SECTION("gait10dof18musc") {
        Model model("walk_gait1018_subject01.osim");
        model.initSystem();

        TableProcessor tableProcessor =
                TableProcessor("walk_gait1018_state_reference.mot") |
                TabOpUseAbsoluteStateNames();
        TimeSeriesTable coordinates =
                tableProcessor.processAndConvertToRadians(model);

        TimeSeriesTable lengths = createMuscleLengthsTable(model, coordinates);

        TimeSeriesTable stdLengths("std_testMuscleLengths_walk_gait1018.sto");
        CHECK(SimTK::Test::numericallyEqual(lengths.getMatrix(),
                stdLengths.getMatrix(),
                static_cast<int>(lengths.getNumColumns()), 1e-6));
    }
}

TEST_CASE("testFunctionBasedPath") {
    
    const double q_x = 0.12;
    const double q_y = 0.34;
    const double qdot_x = 0.56;
    const double qdot_y = 0.78;
    const double tension = 0.91;
    
    SECTION("Sliding point mass, length function") {
        // 1-DOF polynomial path function.
        // length = q^3 + 2*q^2 + 3*q + 4
        PolynomialFunction poly(createVector({1.0, 2.0, 3.0, 4.0}));
        
        // Test values.
        // momentArm = -dl/dq = -3*q^2 - 4*q - 3
        // speed = -qdot * momentArm
        const double length = q_x*q_x*q_x + 2*q_x*q_x + 3*q_x + 4;
        const double momentArm = -3*q_x*q_x - 4*q_x - 3;
        const double speed = -qdot_x * momentArm;
        const double genForce = tension * momentArm;
        
        // Create a sliding mass model and add a PathActuator with a 1-DOF
        // FunctionBasedPath.
        Model model = ModelFactory::createSlidingPointMass();
        
        FunctionBasedPath fbPath;
        fbPath.setName("polynomial_path_1dof");
        fbPath.setLengthFunction(poly);
        fbPath.setCoordinatePaths({"/slider/position"});
        
        auto* actu = new PathActuator();
        actu->set_path(fbPath);
        actu->setName("actuator");
        actu->setOptimalForce(1);
        model.addComponent(actu);
        model.finalizeConnections();
        
        // Initialize the system and set the state and controls.
        SimTK::State state = model.initSystem();
        model.getCoordinateSet()[0].setValue(state, q_x);
        model.getCoordinateSet()[0].setSpeedValue(state, qdot_x);
        model.setControls(state, createVector({tension, 0.0, 0.0}));
        model.realizeAcceleration(state);
        
        // Run inverse dynamics to compute the generalized force applied by the
        // PathActuator.
        auto& matter = model.updMatterSubsystem();
        SimTK::Vector residuals(1, 0.0);
        matter.calcResidualForce(state, 
                createVector({0.0}), 
                SimTK::Vector_<SimTK::SpatialVec>(2, 
                    SimTK::SpatialVec(SimTK::Vec3(0), SimTK::Vec3(0))),
                state.getUDot(), 
                SimTK::Vector(0),
                residuals);
        
        // Check that the length, moment arms, speed, and generalized forces are
        // correct. Compare quantities that should have been calculated to 
        // machine tolerance given the problem size, which we'll characterize by 
        // the number of mobilities (based on Simbody's testing).
        const auto& path = actu->getPath();
        const double tol = 10 * state.getNU() * SimTK::Test::defTol<double>();
        CHECK_THAT(length, WithinAbs(path.getLength(state), tol));
        auto& coord = model.getCoordinateSet()[0];
        CHECK_THAT(momentArm, 
            WithinAbs(path.computeMomentArm(state, coord), tol));
        CHECK_THAT(speed, 
            WithinAbs(path.getLengtheningSpeed(state), tol));
        CHECK_THAT(genForce, WithinAbs(residuals[0], tol));
    }
    
    SECTION("Planar point mass, length function") {
        // 2-DOF polynomial path function.
        // length = 1 + 2*q_y + 3*q_y^2 + 4*q_x + 5*q_x*q_y + 6*q_x^2
        MultivariatePolynomialFunction poly(
                createVector({1.0, 2.0, 3.0, 4.0, 5.0, 6.0}), 2, 2);
        
        // Test values.
        // momentArm_x = -dl/dq_x = -4 - 5*q_y - 12*q_x
        // momentArm_y = -dl/dq_y = -2 - 5*q_x - 6*q_y
        // speed = -qdot_x * momentArm_x - qdot_y * momentArm_y
        const double length = 1.0 + 2.0 * q_y + 3.0 * q_y * q_y + 4.0 * q_x + 
                              5.0 * q_x * q_y + 6.0 * q_x * q_x;
        const double momentArm_x = -4.0 - 5.0 * q_y - 12.0 * q_x;
        const double momentArm_y = -2.0 - 5.0 * q_x - 6.0 * q_y;
        const double speed = -qdot_x * momentArm_x - qdot_y * momentArm_y;
        const double genForce_x = tension * momentArm_x;
        const double genForce_y = tension * momentArm_y;
        
        // Create a planar point mass model and add a PathActuator with a 2-DOF
        // FunctionBasedPath.
        Model model = ModelFactory::createPlanarPointMass();
        model.setGravity(SimTK::Vec3(0.0));
        
        FunctionBasedPath fbPath;
        fbPath.setName("polynomial_path_2dof");
        fbPath.setLengthFunction(poly);
        fbPath.setCoordinatePaths({"/jointset/tx/tx", "/jointset/ty/ty"});
        
        auto* actu = new PathActuator();
        actu->set_path(fbPath);
        actu->setName("actuator");
        actu->setOptimalForce(1);
        model.addComponent(actu);
        model.finalizeConnections();
        
        // Initialize the system and set the state and controls.
        SimTK::State state = model.initSystem();
        model.getCoordinateSet()[0].setValue(state, q_x);
        model.getCoordinateSet()[1].setValue(state, q_y);
        model.getCoordinateSet()[0].setSpeedValue(state, qdot_x);
        model.getCoordinateSet()[1].setSpeedValue(state, qdot_y);
        model.setControls(state, createVector({tension, 0.0, 0.0}));
        model.realizeAcceleration(state);
        
        // Run inverse dynamics to compute the generalized forces applied by the
        // PathActuator.
        auto& matter = model.updMatterSubsystem();
        SimTK::Vector residuals(2, 0.0);
        matter.calcResidualForce(state, 
                SimTK::Vector(2, 0.0), 
                SimTK::Vector_<SimTK::SpatialVec>(3, 
                    SimTK::SpatialVec(SimTK::Vec3(0), SimTK::Vec3(0))),
                state.getUDot(), 
                SimTK::Vector(0),
                residuals);
        
        // Check that the length, moment arms, speed, and generalized forces are
        // correct. Compare quantities that should have been calculated to 
        // machine tolerance given the problem size, which we'll characterize by 
        // the number of mobilities (based on Simbody's testing).
        const auto& path = actu->getPath();
        const double tol = 10 * state.getNU() * SimTK::Test::defTol<double>();
        CHECK_THAT(length, WithinAbs(path.getLength(state), tol));
        auto& tx = model.getCoordinateSet()[0];
        auto& ty = model.getCoordinateSet()[1];
        CHECK_THAT(momentArm_x, 
                WithinAbs(path.computeMomentArm(state, tx), tol));
        CHECK_THAT(momentArm_y, 
                WithinAbs(path.computeMomentArm(state, ty), tol));
        CHECK_THAT(speed, 
                WithinAbs(path.getLengtheningSpeed(state), tol));
        CHECK_THAT(genForce_x, WithinAbs(residuals[0], tol));
        CHECK_THAT(genForce_y, WithinAbs(residuals[1], tol));
    }
    
    SECTION("Planar point mass, all functions") {
        // 2-DOF polynomial path function.
        // length = 1 + 2*q_y + 3*q_y^2 + 4*q_x + 5*q_x*q_y + 6*q_x^2
        MultivariatePolynomialFunction lengthFunc(
                createVector({1.0, 2.0, 3.0, 4.0, 5.0, 6.0}), 2, 2);

        // Moment arm functions.
        // momentArm_x = -dl/dq_x = -4 - 5*q_y - 12*q_x
        // momentArm_y = -dl/dq_y = -2 - 6*q_y - 5*q_x
        MultivariatePolynomialFunction momentArmFunc_x(
                createVector({-4.0, -5.0, -12.0}), 2, 1);
        MultivariatePolynomialFunction momentArmFunc_y(
                createVector({-2.0, -6.0, -5.0}), 2, 1);

        // Speed function.
        // speed = -qdot_x * momentArm_x - qdot_y * momentArm_y
        //       = qdot_x * (4 + 5*q_y + 12*q_x) + qdot_y * (2 + 5*q_x + 6*q_y)
        //       = 4*qdot_x + 5*qdot_x*q_y + 12*qdot_x*q_x + 2*qdot_y +
        //         5*qdot_y*q_x + 6*qdot_y*q_y
        // 
        // See the documentation for MultivariatePolynomialFunction for an
        // explanation of the coefficients.
        SimTK::Vector speedCoeffs(15, 0.0);
        speedCoeffs[1] = 2.0;
        speedCoeffs[3] = 4.0;
        speedCoeffs[7] = 6.0;
        speedCoeffs[8] = 5.0;
        speedCoeffs[11] = 5.0;
        speedCoeffs[12] = 12.0;
        MultivariatePolynomialFunction speedFunc(speedCoeffs, 4, 2);

        // Test values.
        const double length = 1.0 + 2.0 * q_y + 3.0 * q_y * q_y + 4.0 * q_x + 
                              5.0 * q_x * q_y + 6.0 * q_x * q_x;
        const double momentArm_x = -4.0 - 5.0 * q_y - 12.0 * q_x;
        const double momentArm_y = -2.0 - 5.0 * q_x - 6.0 * q_y;
        const double speed = -qdot_x * momentArm_x - qdot_y * momentArm_y;
        const double genForce_x = tension * momentArm_x;
        const double genForce_y = tension * momentArm_y;

        // Create a planar point mass model and add a PathActuator with a 2-DOF
        // FunctionBasedPath.
        Model model = ModelFactory::createPlanarPointMass();
        model.setGravity(SimTK::Vec3(0.0));

        FunctionBasedPath fbPath;
        fbPath.setName("polynomial_path_2dof");
        fbPath.setLengthFunction(lengthFunc);
        fbPath.appendMomentArmFunction(momentArmFunc_x);
        fbPath.appendMomentArmFunction(momentArmFunc_y);
        fbPath.setLengtheningSpeedFunction(speedFunc);
        fbPath.setCoordinatePaths({"/jointset/tx/tx", "/jointset/ty/ty"});

        auto* actu = new PathActuator();
        actu->set_path(fbPath);
        actu->setName("actuator");
        actu->setOptimalForce(1);
        model.addComponent(actu);
        model.finalizeConnections();

        // Initialize the system and set the state and controls.
        SimTK::State state = model.initSystem();
        model.getCoordinateSet()[0].setValue(state, q_x);
        model.getCoordinateSet()[1].setValue(state, q_y);
        model.getCoordinateSet()[0].setSpeedValue(state, qdot_x);
        model.getCoordinateSet()[1].setSpeedValue(state, qdot_y);
        model.setControls(state, createVector({tension, 0.0, 0.0}));
        model.realizeAcceleration(state);

        // Run inverse dynamics to compute the generalized forces applied by the
        // PathActuator.
        auto& matter = model.updMatterSubsystem();
        SimTK::Vector residuals(2, 0.0);
        matter.calcResidualForce(state, 
                SimTK::Vector(2, 0.0), 
                SimTK::Vector_<SimTK::SpatialVec>(3, 
                    SimTK::SpatialVec(SimTK::Vec3(0), SimTK::Vec3(0))),
                state.getUDot(), 
                SimTK::Vector(0),
                residuals);

        // Check that the length, moment arms, speed, and generalized forces are
        // correct. Compare quantities that should have been calculated to 
        // machine tolerance given the problem size, which we'll characterize by 
        // the number of mobilities (based on Simbody's testing).
        const auto& path = actu->getPath();
        const double tol = 10 * state.getNU() * SimTK::Test::defTol<double>();
        CHECK_THAT(length, WithinAbs(path.getLength(state), tol));
        auto& tx = model.getCoordinateSet()[0];
        auto& ty = model.getCoordinateSet()[1];
        CHECK_THAT(momentArm_x,
                WithinAbs(path.computeMomentArm(state, tx), tol));
        CHECK_THAT(momentArm_y, 
                WithinAbs(path.computeMomentArm(state, ty), tol));
        CHECK_THAT(speed, 
                WithinAbs(path.getLengtheningSpeed(state), tol));
        CHECK_THAT(genForce_x, WithinAbs(residuals[0], tol));
        CHECK_THAT(genForce_y, WithinAbs(residuals[1], tol));
    }

    SECTION("Planar point mass, MultivariatePolynoimalFunction helpers") {
        // 2-DOF polynomial path function.
        // length = 1 + 2*q_y + 3*q_y^2 + 4*q_x + 5*q_x*q_y + 6*q_x^2
        MultivariatePolynomialFunction lengthFunc(
                createVector({1.0, 2.0, 3.0, 4.0, 5.0, 6.0}), 2, 2);

        // Moment arm functions.
        // These functions are the first derivative with respect to the
        // corresponding coordinate of the length function. The coefficients are
        // negated to match the convention in OpenSim.
        bool negateCoefficients = true;
        MultivariatePolynomialFunction momentArmFunc_x = 
            lengthFunc.generateDerivativeFunction(0, negateCoefficients);
        MultivariatePolynomialFunction momentArmFunc_y =
            lengthFunc.generateDerivativeFunction(1, negateCoefficients);

        // Speed function.
        // The lengthening speed function is the time derivative of the length
        // length function, which be computed by 
        MultivariatePolynomialFunction speedFunc = 
                lengthFunc.generatePartialVelocityFunction();

        // Test values.
        const double length = 1.0 + 2.0 * q_y + 3.0 * q_y * q_y + 4.0 * q_x + 
                              5.0 * q_x * q_y + 6.0 * q_x * q_x;
        const double momentArm_x = -4.0 - 5.0 * q_y - 12.0 * q_x;
        const double momentArm_y = -2.0 - 5.0 * q_x - 6.0 * q_y;
        const double speed = -qdot_x * momentArm_x - qdot_y * momentArm_y;
        const double genForce_x = tension * momentArm_x;
        const double genForce_y = tension * momentArm_y;

        // Create a planar point mass model and add a PathActuator with a 2-DOF
        // FunctionBasedPath.
        Model model = ModelFactory::createPlanarPointMass();
        model.setGravity(SimTK::Vec3(0.0));

        FunctionBasedPath fbPath;
        fbPath.setName("polynomial_path_2dof");
        fbPath.setLengthFunction(lengthFunc);
        fbPath.appendMomentArmFunction(momentArmFunc_x);
        fbPath.appendMomentArmFunction(momentArmFunc_y);
        fbPath.setLengtheningSpeedFunction(speedFunc);
        fbPath.setCoordinatePaths({"/jointset/tx/tx", "/jointset/ty/ty"});

        auto* actu = new PathActuator();
        actu->set_path(fbPath);
        actu->setName("actuator");
        actu->setOptimalForce(1);
        model.addComponent(actu);
        model.finalizeConnections();

        // Initialize the system and set the state and controls.
        SimTK::State state = model.initSystem();
        model.getCoordinateSet()[0].setValue(state, q_x);
        model.getCoordinateSet()[1].setValue(state, q_y);
        model.getCoordinateSet()[0].setSpeedValue(state, qdot_x);
        model.getCoordinateSet()[1].setSpeedValue(state, qdot_y);
        model.setControls(state, createVector({tension, 0.0, 0.0}));
        model.realizeAcceleration(state);

        // Run inverse dynamics to compute the generalized forces applied by the
        // PathActuator.
        auto& matter = model.updMatterSubsystem();
        SimTK::Vector residuals(2, 0.0);
        matter.calcResidualForce(state, 
                SimTK::Vector(2, 0.0), 
                SimTK::Vector_<SimTK::SpatialVec>(3, 
                    SimTK::SpatialVec(SimTK::Vec3(0), SimTK::Vec3(0))),
                state.getUDot(), 
                SimTK::Vector(0),
                residuals);

        // Check that the length, moment arms, speed, and generalized forces are
        // correct. Compare quantities that should have been calculated to 
        // machine tolerance given the problem size, which we'll characterize by 
        // the number of mobilities (based on Simbody's testing).
        const auto& path = actu->getPath();
        const double tol = 10 * state.getNU() * SimTK::Test::defTol<double>();
        CHECK_THAT(length, WithinAbs(path.getLength(state), tol));
        auto& tx = model.getCoordinateSet()[0];
        auto& ty = model.getCoordinateSet()[1];
        CHECK_THAT(momentArm_x,
                WithinAbs(path.computeMomentArm(state, tx), tol));
        CHECK_THAT(momentArm_y, 
                WithinAbs(path.computeMomentArm(state, ty), tol));
        CHECK_THAT(speed, 
                WithinAbs(path.getLengtheningSpeed(state), tol));
        CHECK_THAT(genForce_x, WithinAbs(residuals[0], tol));
        CHECK_THAT(genForce_y, WithinAbs(residuals[1], tol));
    }

}
