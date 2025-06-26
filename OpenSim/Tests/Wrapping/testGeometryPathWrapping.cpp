/* -------------------------------------------------------------------------- *
 *                   OpenSim:  testGeometryPathWrapping.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Ayman Habib, Nicholas Bianco                         *
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

#include <OpenSim/Common/Constant.h> 
#include <OpenSim/Simulation/StatesTrajectory.h>
#include <OpenSim/Simulation/TableProcessor.h>
#include <OpenSim/Simulation/Control/PrescribedController.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PathSpring.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/Wrap/WrapObject.h>
#include <OpenSim/Simulation/Wrap/WrapCylinder.h>
#include <OpenSim/Simulation/Wrap/WrapSphere.h>
#include <OpenSim/Simulation/Wrap/WrapEllipsoid.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <catch2/catch_all.hpp>

using namespace OpenSim;

using Catch::Matchers::WithinAbs;

// Namespace for GeometryPath simulation test helper functions.
namespace {
    // Simulate a model from an initial time to a final time given an initial
    // state.
    void simulate(Model& model, SimTK::State& state, double initialTime, 
            double finalTime) {
        // Create the Manager for the simulation.
        const double accuracy = 1.0e-4;
        Manager manager(model);
        manager.setIntegratorAccuracy(accuracy);
    
        // Integrate from initial time to final time.
        state.setTime(initialTime);
        std::cout << "\nIntegrating from " << initialTime << " to " 
             << finalTime << std::endl;
    
        const double start = SimTK::realTime();
        manager.initialize(state);
        manager.integrate(finalTime);
        std::cout << "Simulation time = " << SimTK::realTime() - start
             << " seconds (wallclock time)\n"
             << std::endl;
    
        auto& integrator = manager.getIntegrator();
        std::cout << "Integrator iterations = " 
             << integrator.getNumStepsTaken() << std::endl;
    
        // Save the simulation results.
        Storage states(manager.getStateStorage());
        states.print(model.getName() + "_states.sto");
        model.updSimbodyEngine().convertRadiansToDegrees(states);
        states.setWriteSIMMHeader(true);
        states.print(model.getName() + "_states_degrees.mot");
    }

    // Simulate a model with muscles at a constant activation to a given final
    // time.
    void simulateModelWithMuscles(const std::string& modelFile, double finalTime, 
            double activation = 0.5) {
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
        auto statesTraj = StatesTrajectory::createFromStatesTable(
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

// Namespace for single WrapObject helper functions.
namespace {
    constexpr double radius = 0.5;

    // Test results of wrapping a single path perpendicular to a WrapObject,
    // particularly a path perpendicular to cylinder axis (Z axis), and compare 
    // the results to analytical/expected answers. Since cross-section is a 
    // circle/arc, results should match a sphere or ellipsoid with matching 
    // radius. In the ground fram, the path is in XY plane along x-axis tangent 
    // the WrapObject and wraps with coordinate change.
    void testSingleWrapObjectPerpendicular(WrapObject* wrapObject, 
            SimTK::Vec3 axisRotations) {
        auto visualize = false;
        const double r = radius;
        Model model;
        model.setName("test"+wrapObject->getConcreteClassName());

        auto& ground = model.updGround();
        auto body = new OpenSim::Body("body", 1, SimTK::Vec3(-r, 0, 0), 
                SimTK::Inertia(0.1, 0.1, 0.01));
        model.addComponent(body);
        
        auto joint = new PinJoint("pin", ground, *body);
        auto& qi = joint->updCoordinate();
        qi.setName("q_pin");
        model.addComponent(joint);

        // Add the wrap object to the body, which takes ownership of it
        WrapObject* wObj = wrapObject->clone();
        wObj->set_xyz_body_rotation(axisRotations);
        ground.addWrapObject(wObj);

        // One spring has wrap cylinder with respect to ground origin
        PathSpring* spring1 =
            new PathSpring("spring1", 1.0, 0.1, 0.01);
        // Offset in X direction to avoid ambiguous scenario where path passes 
        // through center.
        spring1->updGeometryPath().
            appendNewPathPoint("origin", ground, SimTK::Vec3(r-.1, r, 0)); 
        spring1->updGeometryPath().
            appendNewPathPoint("insert", *body, SimTK::Vec3(-r, r, 0));
        spring1->updGeometryPath().addPathWrap(*wObj);

        model.addComponent(spring1);

        model.finalizeConnections();
        model.setUseVisualizer(visualize);
        //model.print(wObj->getConcreteClassName()+"Analytical.osim");
        //model.updDisplayHints().disableVisualization();
        SimTK::State& s = model.initSystem();
        auto& coord = joint->getCoordinate();
        int nsteps = 1000;
        for (int i = 0; i <= nsteps; ++i) {
            
            coord.setValue(s, i*SimTK::Pi/(2*nsteps));
            model.realizeVelocity(s);

            if (visualize)
                model.getVisualizer().show(s);

            double ma1 = spring1->computeMomentArm(s, coord);

            ASSERT_EQUAL<double>(-r, ma1, .0001); // SimTK::Eps
            double len1 = spring1->getLength(s);
            // Length is 2*r -0.1 by construction plus a portion of a quarter 
            // circle with radius r proportional to i.
            ASSERT_EQUAL<double>(len1, 
                    2*r-0.1 + 0.25 * 2 * SimTK::Pi * r * i / nsteps, 1e-6); 

        }
    }

    // Ellipsoid passed in has radii of a, b, c wrapping occurs along z axis
    // no closed-form analytical solution to compare but approximate length
    // for full ellipse. Wrapping should match approximately 1/4 
    // perimeter of ellipse + fixed offset baked in.
    void testEllipsoidWrapLength(WrapEllipsoid* wrapObject) {
        auto visualize = false;
        const double r = radius;
        Model model;

        auto& ground = model.updGround();
        auto body = new OpenSim::Body("body", 1, SimTK::Vec3(-r, 0, 0), 
                SimTK::Inertia(0.1, 0.1, 0.01));
        model.addComponent(body);

        auto joint = new PinJoint("pin", ground, *body);
        auto& qi = joint->updCoordinate();
        qi.setName("q_pin");
        model.addComponent(joint);

        // Add the wrap object to the body, which takes ownership of it
        WrapObject* wObj = wrapObject->clone();
        ground.addWrapObject(wObj);

        // One spring has wrap cylinder with respect to ground origin
        PathSpring* spring1 =
            new PathSpring("spring1", 1.0, 0.1, 0.01);
        // Offset in X direction to avoid ambiguous scenario where path passes 
        // through center.
        spring1->updGeometryPath().appendNewPathPoint("origin", ground, 
                SimTK::Vec3(r - .1, r, 0)); 
        // Insertion point is -r down from the tip of the long axis of the 
        // ellipsoid.
        spring1->updGeometryPath().appendNewPathPoint("insert", *body, 
                SimTK::Vec3(-wrapObject->get_dimensions()[0], -r, 0));
        spring1->updGeometryPath().addPathWrap(*wObj);

        model.addComponent(spring1);

        model.finalizeConnections();
        model.setUseVisualizer(visualize);

        SimTK::State& s = model.initSystem();
        model.realizeVelocity(s);

        if (visualize)
            model.getVisualizer().show(s);

        double len1 = spring1->getLength(s);
        // Reference Ramanujan formula:
        // https://www.cuemath.com/measurement/perimeter-of-ellipse/
        double a = wrapObject->get_dimensions()[0];
        double b = wrapObject->get_dimensions()[1];
        double h = ((a - b) * (a - b)) / ((a + b) * (a+b));
        double lengthAnalyticalApprox = 
                SimTK::Pi * (a + b) * (1 + 3 * h / (10 + std::sqrt(4 - 3 * h)));
        // Length is 1/4 ellipse + 2r -.1.
        ASSERT_EQUAL<double>(len1, 2 * r - 0.1 + lengthAnalyticalApprox/4, 1e-4);

    }

}

TEST_CASE("PathSpring with WrapCylinder") {
    const double r = 0.25;
    const double off = sqrt(2)*r-0.05;
    Model model;
    model.setName("testWrapCylinder");

    auto& ground = model.updGround();
    auto body = new OpenSim::Body("body", 1, SimTK::Vec3(0), 
            SimTK::Inertia(0.1, 0.1, 0.01));
    model.addComponent(body);

    auto bodyOffset = new PhysicalOffsetFrame(
            "bToj", *body, SimTK::Transform(SimTK::Vec3(-off, 0, 0)));
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
        appendNewPathPoint("origin", ground, SimTK::Vec3(-off, 0, 0));
    spring1->updGeometryPath().
        appendNewPathPoint("insert", *body, SimTK::Vec3(0));
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
        appendNewPathPoint("origin", ground, SimTK::Vec3(-off, 0, 0));
    spring2->updGeometryPath().
        appendNewPathPoint("insert", *body, SimTK::Vec3(0));
    spring2->updGeometryPath().addPathWrap(*pulley2);
    spring2->updGeometryPath().setDefaultColor(SimTK::Vec3(0, 0.8, 0));

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

TEST_CASE("Shoulder wrapping performance") {
    // Test the performance of multiple paths with wrapping in the 
    // upper-extremity.
    const double finalTime = 0.1;
    const double activation = 0.5;
    simulateModelWithMuscles(
            "testGeometryPathWrapping_ShoulderWrappingWithPathSprings.osim", 
            finalTime, activation);
}

TEST_CASE("Muscle lengths") {
    
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
                "std_testGeometryPathWrapping_subject_walk_armless_muscle_lengths.sto");
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

        TimeSeriesTable stdLengths(
                "std_testGeometryPathWrapping_walk_gait1018_muscle_lengths.sto");
        CHECK(SimTK::Test::numericallyEqual(lengths.getMatrix(),
                stdLengths.getMatrix(),
                static_cast<int>(lengths.getNumColumns()), 1e-6));
    }
}

TEST_CASE("WrapCylinder") {
    auto* wo = new WrapCylinder();
    wo->setName("pulley1");
    wo->set_radius(radius);
    wo->set_length(1);
    
    testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0));
    // Rotating a cylinder around its axis doesn't change wrapping result but
    // changes the local coordinate system for computation by changing the 
    // quadrant.
    testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0, 0, SimTK::Pi / 2));
    testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0, 0, SimTK::Pi));
    testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0, 0, -SimTK::Pi / 2));
    wo->set_quadrant("+y");
    testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0));
}

TEST_CASE("WrapSphere") {
    auto* wo = new WrapSphere();
    wo->setName("pulley1");
    wo->set_radius(radius);
    testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0));
    testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0, 0, SimTK::Pi / 2));
    testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0, 0, SimTK::Pi));
    testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0, 0, -SimTK::Pi / 2));
    testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0, SimTK::Pi / 2, 0));
    testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0, SimTK::Pi, 0));
    testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0, -SimTK::Pi / 2, 0));
    testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(SimTK::Pi / 2, 0, 0));
    testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(SimTK::Pi, 0, 0));
    testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(-SimTK::Pi / 2, 0, 0));
    wo->set_quadrant("+y");
    testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0));
}

TEST_CASE("WrapEllipsoid") {
    SECTION("Perpendicular wrapping") {
        auto* wo = new WrapEllipsoid();
        wo->setName("pulley1");
        // Use wrapEllipsoid methods to wrap on a sphere
        wo->set_dimensions(SimTK::Vec3(radius));
        testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0));
        testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0, 0, SimTK::Pi/2));
        testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0, 0, SimTK::Pi));
        testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0, 0, -SimTK::Pi/2));
        wo->set_quadrant("+y");
        testSingleWrapObjectPerpendicular(wo, SimTK::Vec3(0));
    }
    SECTION("Wrapping length") {
        using SimTK::Pi;
        auto* wo = new WrapEllipsoid();
        wo->setName("pulley1");
        // Change rotation angle by 1 deg up to a little under pi/2 which is a 
        // singularity.
        for (double angle = 0; angle < Pi/2 -.1; angle += Pi / 180*5) {
            wo->set_dimensions(SimTK::Vec3(radius / cos(angle), radius, 1));
            testEllipsoidWrapLength(wo);
        }
    }
}
