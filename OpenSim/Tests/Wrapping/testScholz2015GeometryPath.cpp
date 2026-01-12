/* -------------------------------------------------------------------------- *
 *                  OpenSim:  testScholz2015GeometryPath.cpp                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
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

#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Simulation/Control/PrescribedController.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PathSpring.h>
#include <OpenSim/Simulation/Model/Scholz2015GeometryPath.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>

#include <OpenSim/Simulation/VisualizerUtilities.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;

TEST_CASE("Interface") {
    Model model = ModelFactory::createDoublePendulum();

    Scholz2015GeometryPath* path = new Scholz2015GeometryPath();
    path->setName("path");
    model.addComponent(path);

    SECTION("No path points") {
        // Adding a component calls finalizeFromProperties() internally.
        CHECK_THROWS_WITH(model.initSystem(),
            Catch::Matchers::ContainsSubstring(
                "Expected at least two path points before finalizing the path, "
                "but 0 path point(s) were found."));
    }

    // Add the first path point. This defines the origin of the path.
    path->appendPathPoint(model.getGround(), SimTK::Vec3(0.05, 0.05, 0.));

    SECTION("Only one path point") {
        CHECK_THROWS_WITH(model.initSystem(),
            Catch::Matchers::ContainsSubstring(
                "Expected at least two path points before finalizing the path, "
                "but 1 path point(s) were found."));
    }

    // Append a second path point, creating a straight line segment between the
    // first and second path points.
    path->appendPathPoint(model.getComponent<Body>("/bodyset/b0"),
            SimTK::Vec3(-0.75, 0.1, 0.));

    // Append a ContactCylinder wrapping obstacle to the path.
    auto* obstacle = new ContactCylinder(0.1,
        SimTK::Vec3(-0.5, 0.1, 0.), SimTK::Vec3(0),
        model.getComponent<Body>("/bodyset/b0"));
    model.addComponent(obstacle);
    path->appendObstacle(*obstacle, SimTK::Vec3(0., 0.1, 0.));

    SECTION("Path not closed") {
        CHECK_THROWS_WITH(model.initSystem(),
            Catch::Matchers::ContainsSubstring(
               "Expected the last element of the path to be a path "
               "point, but it is an obstacle."));
    }

    // Add additional path points.
    path->appendPathPoint(model.getComponent<Body>("/bodyset/b1"),
            SimTK::Vec3(-0.25, 0.1, 0.));

    SECTION("Valid path") {
        CHECK_NOTHROW(model.initSystem());
    }

    SECTION("Path configuration") {
        CHECK(path->getNumPathPoints() == 3);
        CHECK(path->getNumObstacles() == 1);
        CHECK(path->getNumPathElements() == 4);
    }

    SECTION("Accessing path points") {
        CHECK_NOTHROW(path->getPathPoint(0));
        CHECK_NOTHROW(path->getPathPoint(1));
        CHECK_NOTHROW(path->getPathPoint(2));
        CHECK_THROWS_WITH(path->getPathPoint(3),
            Catch::Matchers::ContainsSubstring(
                "Index 3 is out of range."));
    }

    SECTION("Accessing obstacles") {
        CHECK_NOTHROW(path->getContactGeometry(0));
        CHECK_THROWS_WITH(path->getContactGeometry(1),
            Catch::Matchers::ContainsSubstring(
                "Index 1 is out of range."));
        CHECK_NOTHROW(path->getContactHint(0));
        CHECK_THROWS_WITH(path->getContactHint(1),
            Catch::Matchers::ContainsSubstring(
                "Index 1 is out of range."));
    }

    SECTION("Serialization and deserialization") {
        model.finalizeConnections();
        std::string fileName = "testScholz2015GeometryPath.osim";
        model.print(fileName);

        Model model2(fileName);
        CHECK(model == model2);
    }
}

TEST_CASE("Suspended pendulum") {

    // Create a single pendulum model.
    Model model = ModelFactory::createPendulum();

    // Add a path spring that will wrap over each ContactGeometry, suspending
    // the pendulum. We use a relatively large dissipation coefficient to ensure
    // that the pendulum comes to rest quickly.
    const double stiffness = 25.0;
    const double dissipation = 5.0;
    auto* actu = new PathSpring();
    actu->setName("path_spring");
    actu->setRestingLength(0.0);
    actu->setDissipation(dissipation);
    actu->setStiffness(stiffness);
    actu->set_path(Scholz2015GeometryPath());
    model.addComponent(actu);

    // Set the path's origin and insertion.
    Scholz2015GeometryPath& path = actu->updPath<Scholz2015GeometryPath>();
    path.appendPathPoint(model.getGround(), SimTK::Vec3(-0.1, 0, 0));

    // Check that pendulum is at rest with the expected length.
    const double expectedLength = 0.81268778;
    const double expectedSpeed = 0.0;
    const double expectedTension = stiffness * expectedLength +
            dissipation * expectedSpeed;
    auto simulateHangingPendulum = [&](Model& model) {
        SimTK::State state = model.initSystem();
        Manager manager(model);
        manager.initialize(state);
        state = manager.integrate(20.0);

        model.realizeVelocity(state);
        CHECK_THAT(path.getLength(state),
            Catch::Matchers::WithinRel(expectedLength, 1e-3));
        CHECK_THAT(path.getLengtheningSpeed(state),
            Catch::Matchers::WithinAbs(0.0, 1e-3));
        CHECK_THAT(state.getU()[0],
            Catch::Matchers::WithinAbs(0.0, 1e-3));
        CHECK_THAT(actu->getTension(state),
            Catch::Matchers::WithinRel(expectedTension, 1e-3));
    };

    // Simulate the "suspended" pendulum over each contact geometry. The
    // geometries are configured such that each test will result in the same
    // final path length.

    SECTION("ContactCylinder") {
        auto* obstacle = new ContactCylinder(0.1,
            SimTK::Vec3(0.25, 0, 0), SimTK::Vec3(0), model.getGround());
        model.addComponent(obstacle);
        path.appendObstacle(*obstacle, SimTK::Vec3(0.0, 0.1, 0.0));
        path.appendPathPoint(model.getComponent<Body>("/bodyset/b0"),
            SimTK::Vec3(-0.5, 0.1, 0));
        simulateHangingPendulum(model);
    }

    SECTION("ContactEllipsoid") {
        auto* obstacle = new ContactEllipsoid(SimTK::Vec3(0.1, 0.1, 0.3),
            SimTK::Vec3(0.25, 0, 0), SimTK::Vec3(0), model.getGround());
        model.addComponent(obstacle);
        path.appendObstacle(*obstacle, SimTK::Vec3(0.0, 0.1, 0.0));
        path.appendPathPoint(model.getComponent<Body>("/bodyset/b0"),
            SimTK::Vec3(-0.5, 0.1, 0));
        simulateHangingPendulum(model);
    }

    SECTION("ContactSphere") {
        auto* obstacle = new ContactSphere(0.1,
            SimTK::Vec3(0.25, 0, 0), model.getGround());
        model.addComponent(obstacle);
        path.appendObstacle(*obstacle, SimTK::Vec3(0.0, 0.1, 0.0));
        path.appendPathPoint(model.getComponent<Body>("/bodyset/b0"),
            SimTK::Vec3(-0.5, 0.1, 0));
        simulateHangingPendulum(model);
    }

    SECTION("ContactTorus") {
        auto* obstacle = new ContactTorus(0.4, 0.1,
            SimTK::Vec3(0.25, 0.4, 0),
            SimTK::Vec3(0, 0.5*SimTK::Pi, 0),
            model.getGround());
        model.addComponent(obstacle);
        path.appendObstacle(*obstacle, SimTK::Vec3(0.0, -0.3, 0.0));
        path.appendPathPoint(model.getComponent<Body>("/bodyset/b0"),
            SimTK::Vec3(-0.5, 0.1, 0));
        simulateHangingPendulum(model);
    }
}

TEST_CASE("Moment arms") {

    const SimTK::Real radius = 0.2;
    const SimTK::Real offset = 0.5;

    // Construct a double pendulum model.
    Model model = ModelFactory::createDoublePendulum();

    // Configure the Scholz2015GeometryPath.
    Scholz2015GeometryPath* path = new Scholz2015GeometryPath();
    path->setName("path");
    path->appendPathPoint(model.getComponent<Body>("/bodyset/b0"),
            SimTK::Vec3(-offset, 0., 0.));
    // Add a ContactCylinder along the axis definiing the PinJoint between
    // body 'b0' and body 'b1'.
    auto* obstacle = new ContactCylinder(radius,
        SimTK::Vec3(0., 0., 0.), SimTK::Vec3(0),
        model.getComponent<Body>("/bodyset/b0"));
    model.addComponent(obstacle);
    path->appendObstacle(*obstacle, SimTK::Vec3(0., radius, 0.));
    path->appendPathPoint(model.getComponent<Body>("/bodyset/b1"),
            SimTK::Vec3(-offset, 0., 0.));

    // Add the path to the model.
    model.addComponent(path);

    // The model is in the default configuration with both bodies aligned along
    // their x-axes. Check that the computed moment arm is equal to the cylinder
    // radius.
    SECTION("Zero degrees") {
        SimTK::State state = model.initSystem();
        model.realizePosition(state);
        const Coordinate& coord = model.getCoordinateSet().get("q1");
        CHECK_THAT(path->computeMomentArm(state, coord),
                Catch::Matchers::WithinRel(radius, 1e-6));
    }

    // The model is now posed with the second body rotated 90 degrees relative
    // to the first body. The path should be lifted off the cylinder, no matter
    // what cylinder radius is used in the range defined above.
    SECTION("90 degrees") {
        model.updCoordinateSet().get("q1").setDefaultValue(SimTK::Pi/2.);
        SimTK::State state = model.initSystem();
        model.realizePosition(state);
        const Coordinate& coord = model.getCoordinateSet().get("q1");

        SimTK::Real pathLength = std::sqrt(2*offset*offset);
        CHECK_THAT(path->getLength(state),
                Catch::Matchers::WithinRel(pathLength, 1e-6));

        SimTK::Real momentArm =
            std::sqrt(offset*offset - 0.25*pathLength*pathLength);
        CHECK_THAT(path->computeMomentArm(state, coord),
                Catch::Matchers::WithinRel(momentArm, 1e-6));
    }

    // The model is now posed with the second body rotated 180 degrees relative
    // to the first body, such that the two bodies are fully overlapping. In
    // this configuration, the path length should be zero and the moment arm
    // should be equal to the offset used for the origin and insertion points.
    SECTION("180 degrees") {
        model.updCoordinateSet().get("q1").setDefaultValue(SimTK::Pi);
        SimTK::State state = model.initSystem();
        model.realizePosition(state);
        const Coordinate& coord = model.getCoordinateSet().get("q1");

        CHECK_THAT(path->getLength(state),
                Catch::Matchers::WithinAbs(0.0, 1e-6));
        CHECK_THAT(path->computeMomentArm(state, coord),
                Catch::Matchers::WithinRel(offset, 1e-6));
    }
}

// Test a simple Scholz2015GeometryPath with a known solution. The path wraps
// over the following obstacles, in order: torus, ellipsoid, torus, cylinder.
// We wrap the cable conveniently over the obstacles such that each curve
// segment becomes a circular-arc shape. This allows us to check the results by
// hand.
//
// This test is based on "testSimpleCable" in Simbody's TestCableSpan.cpp.
TEST_CASE("Simple path") {

    // Default mass properties.
    const SimTK::Real mass = 1.0;
    const SimTK::Vec3 com(0);
    const SimTK::Inertia inertia(1);

    // Create a model and add path-related bodies.
    Model model;
    auto* originBody = new Body("origin", mass, com, inertia);
    auto* insertionBody = new Body("insertion", mass, com, inertia);
    auto* torusBody = new Body("torus_body", mass, com, inertia);
    auto* ellipsoidBody = new Body("ellipsoid_body", mass, com, inertia);
    model.addBody(originBody);
    model.addBody(insertionBody);
    model.addBody(torusBody);
    model.addBody(ellipsoidBody);

    model.addJoint(new FreeJoint("origin_joint",
            model.getGround(), SimTK::Vec3(0.), SimTK::Vec3(0.),
            *originBody, SimTK::Vec3(0.), SimTK::Vec3(0.)));
    model.addJoint(new FreeJoint("insertion_joint",
            model.getGround(), SimTK::Vec3(-4., 0., 0.), SimTK::Vec3(0.),
            *insertionBody, SimTK::Vec3(0.), SimTK::Vec3(0.)));
    model.addJoint(new FreeJoint("torus_joint",
            model.getGround(), SimTK::Vec3(0., 10., 0.), SimTK::Vec3(0.),
            *torusBody, SimTK::Vec3(0.), SimTK::Vec3(0.)));
    model.addJoint(new FreeJoint("ellipsoid_joint",
            *torusBody, SimTK::Vec3(4., -10., 0.), SimTK::Vec3(0.),
            *ellipsoidBody, SimTK::Vec3(0.), SimTK::Vec3(0.)));

    // Create a PathActuator with a Scholz2015GeometryPath.
    auto* actu = new PathActuator();
    actu->setName("path_actuator");
    actu->setOptimalForce(1.0);
    actu->set_path(Scholz2015GeometryPath());
    model.addComponent(actu);

    // Add a prescribed controller for the actuator.
    auto* controller = new PrescribedController();
    controller->setName("controller");
    model.addController(controller);

    // Configure the path.
    Scholz2015GeometryPath& path = actu->updPath<Scholz2015GeometryPath>();
    path.appendPathPoint(*originBody, SimTK::Vec3(0.));

    // Add the first torus obstacle.
    auto* torus1 = new ContactTorus(10., 1.,
        SimTK::Vec3(1., 1., 0.), SimTK::Vec3(0., 0.5*SimTK::Pi, 0.),
        *torusBody);
    torus1->setName("torus1");
    model.addComponent(torus1);
    path.appendObstacle(*torus1, SimTK::Vec3(0., -9., 0.));

    // Add the ellipsoid obstacle.
    auto* ellipsoid = new ContactEllipsoid(SimTK::Vec3(1., 1., 6.),
        SimTK::Vec3(0.5, 1., 0.), SimTK::Vec3(0), *ellipsoidBody);
    ellipsoid->setName("ellipsoid");
    model.addComponent(ellipsoid);
    path.appendObstacle(*ellipsoid, SimTK::Vec3(1., 1., 0.5));

    // Add the second torus obstacle.
    auto* torus2 = new ContactTorus(10., 1.5,
        SimTK::Vec3(4., -12., 0.), SimTK::Vec3(0., 0.5*SimTK::Pi, 0.),
        model.getGround());
    torus2->setName("torus2");
    model.addComponent(torus2);
    path.appendObstacle(*torus2, SimTK::Vec3(0., 1., 0.));

    // Add the cylinder obstacle.
    auto* cylinder = new ContactCylinder(2.,
        SimTK::Vec3(-2., -1.5, 0.), SimTK::Vec3(0), model.getGround());
    cylinder->setName("cylinder");
    model.addComponent(cylinder);
    path.appendObstacle(*cylinder, SimTK::Vec3(0., -1., 0.));

    // Add the insertion path point.
    path.appendPathPoint(*insertionBody, SimTK::Vec3(0.));

    // Initialize the system and state.
    SimTK::State state = model.initSystem();
    model.realizePosition(state);

    SECTION("Configuration and smoothness") {
        CHECK(path.getNumObstacles() == 4);
        CHECK(path.getNumPathPoints() == 2);
    }

    SECTION("Path length") {
        // Note that the length deviates because the path is solved up to angle
        // tolerance, not length tolerance.
        const SimTK::Real lengthTolerance = 1e-5;

        // Sum all straight line segment lengths + curve line segment lengths.
        const SimTK::Real sumStraightLineSegmentLengths =
                1. + 3.5 + 3. + 6. + 1.5;
        const SimTK::Real sumCurveLineSegmentLengths =
                0.5 * SimTK::Pi * (1. + 1. + 1.5 + 2.);
        const SimTK::Real expectedTotalPathLength =
                sumStraightLineSegmentLengths + sumCurveLineSegmentLengths;
        const SimTK::Real gotTotalPathLength = path.getLength(state);
        CHECK_THAT(gotTotalPathLength, Catch::Matchers::WithinRel(
                expectedTotalPathLength, lengthTolerance));
    }

    SECTION("Body forces exerted by the path") {
        SimTK::Vector_<SimTK::SpatialVec> expectedBodyForces(
                5, {{0., 0., 0.}, {0., 0., 0.}});
        // Ground body - Sphere and cylinder are connected to it.
        expectedBodyForces[0] = SimTK::SpatialVec{{0., 0., 1.5}, {0., 2., 0.}};
        // Cable origin body.
        expectedBodyForces[1] = SimTK::SpatialVec{{0., 0., 0.}, {0., 1., 0.}};
        // Cable termination body.
        expectedBodyForces[2] = SimTK::SpatialVec{{0., 0., 0.}, {0., -1., 0.}};
        // Torus body.
        expectedBodyForces[3] = SimTK::SpatialVec{{0., 0., 8.}, {1., -1., 0.}};
        // Ellipsoid body.
        expectedBodyForces[4] =
                SimTK::SpatialVec{{0., 0., 0.5}, {-1., -1., 0.}};

        // Apply a constant tension and check that the expected body forces are
        // correct.
        const SimTK::Real tension = SimTK::Pi;
        controller->addActuator(*actu);
        controller->prescribeControlForActuator(
                "path_actuator", Constant(tension));
        model.finalizeConnections();
        model.realizeDynamics(state);

        // Subtract out the gravitational forces to get only the forces due to
        // the cable tension.
        SimTK::Vector_<SimTK::SpatialVec> bodyForces =
                model.getRigidBodyForces(state) -
                model.getGravityBodyForces(state);
        for (int i = 0; i < bodyForces.size(); ++i) {
            CHECK_THAT((expectedBodyForces[i] * tension - bodyForces[i]).norm(),
                Catch::Matchers::WithinAbs(0., 1e-5));
        }
    }
}

// Simple Scholz2015GeometryPath comprised of via points with a known solution.
//
// This test is based on "testViaPoints" in Simbody's TestCableSpan.cpp.
TEST_CASE("Via points") {

    // Default mass properties.
    const SimTK::Real mass = 1.0;
    const SimTK::Vec3 com(0);
    const SimTK::Inertia inertia(1);

    // Create a model and add path-related bodies.
    Model model;
    auto* originBody = new Body("origin", mass, com, inertia);
    auto* insertionBody = new Body("insertion", mass, com, inertia);
    auto* viaPoint1Body = new Body("via_point_1", mass, com, inertia);
    auto* viaPoint2Body = new Body("via_point_2", mass, com, inertia);
    auto* viaPoint3Body = new Body("via_point_3", mass, com, inertia);
    model.addBody(originBody);
    model.addBody(insertionBody);
    model.addBody(viaPoint1Body);
    model.addBody(viaPoint2Body);
    model.addBody(viaPoint3Body);

    model.addJoint(new FreeJoint("origin_joint",
            model.getGround(), SimTK::Vec3(0.), SimTK::Vec3(0.),
            *originBody, SimTK::Vec3(0.), SimTK::Vec3(0.)));
    model.addJoint(new FreeJoint("insertion_joint",
            model.getGround(), SimTK::Vec3(-4., 0., 0.), SimTK::Vec3(0.),
            *insertionBody, SimTK::Vec3(0.), SimTK::Vec3(0.)));
    model.addJoint(new FreeJoint("via_point_1_joint",
            model.getGround(), SimTK::Vec3(0., 1., 0.), SimTK::Vec3(0.),
            *viaPoint1Body, SimTK::Vec3(0.), SimTK::Vec3(0.)));
    model.addJoint(new FreeJoint("via_point_2_joint",
            model.getGround(), SimTK::Vec3(0., 1., 2.), SimTK::Vec3(0.),
            *viaPoint2Body, SimTK::Vec3(0.), SimTK::Vec3(0.)));
    model.addJoint(new FreeJoint("via_point_3_joint",
            model.getGround(), SimTK::Vec3(-4., 1., 2.), SimTK::Vec3(0.),
            *viaPoint3Body, SimTK::Vec3(0.), SimTK::Vec3(0.)));

    // Create a PathActuator with a Scholz2015GeometryPath.
    auto* actu = new PathActuator();
    actu->setName("path_actuator");
    actu->setOptimalForce(1.0);
    actu->set_path(Scholz2015GeometryPath());
    model.addComponent(actu);

    // Add a prescribed controller for the actuator.
    auto* controller = new PrescribedController();
    controller->setName("controller");
    model.addController(controller);

    // Configure the path.
    Scholz2015GeometryPath& path = actu->updPath<Scholz2015GeometryPath>();
    path.appendPathPoint(*originBody, SimTK::Vec3(0.));
    path.appendPathPoint(*viaPoint1Body, SimTK::Vec3(0.));
    path.appendPathPoint(*viaPoint2Body, SimTK::Vec3(0.));
    path.appendPathPoint(*viaPoint3Body, SimTK::Vec3(0.));
    path.appendPathPoint(*insertionBody, SimTK::Vec3(0.));

    // Initialize the system and state.
    SimTK::State state = model.initSystem();
    model.realizePosition(state);

    SECTION("Configuration and smoothness") {
        CHECK(path.getNumObstacles() == 0);
        CHECK(path.getNumPathPoints() == 5);
    }

    SECTION("Path length") {
        // Sum all straight line segment lengths.
        const SimTK::Real lengthTolerance = 1e-5;
        const SimTK::Real expectedTotalCableLength =
                1. + 2. + 4. + std::sqrt(5.);
        const SimTK::Real gotTotalCableLength = path.getLength(state);
        CHECK_THAT(gotTotalCableLength, Catch::Matchers::WithinRel(
                expectedTotalCableLength, lengthTolerance));
    }

    SECTION("Body forces exerted by the path") {
        SimTK::Vector_<SimTK::SpatialVec> expectedBodyForces(
                6, {{0., 0., 0.}, {0., 0., 0.}});
        // Ground body.
        expectedBodyForces[0] = SimTK::SpatialVec{{0., 0., 0}, {0., 0., 0.}};
        // Cable origin body.
        expectedBodyForces[1] = SimTK::SpatialVec{{0., 0., 0.}, {0., 1., 0.}};
        // Cable termination body.
        expectedBodyForces[2] = SimTK::SpatialVec{{0., 0., 0.},
                {0., 1./std::sqrt(5.), 2./std::sqrt(5.)}};
        // First via point.
        expectedBodyForces[3] = SimTK::SpatialVec{{0., 0., 0.}, {0., -1., 1.}};
        // Second via point.
        expectedBodyForces[4] = SimTK::SpatialVec{{0., 0., 0.}, {-1., 0, -1.}};
        // Third via point.
        expectedBodyForces[5] = SimTK::SpatialVec{{0., 0., 0.},
                {1., -1./std::sqrt(5.), -2./std::sqrt(5.)}};

        // Apply a constant tension and check that the expected body forces are
        // correct.
        const SimTK::Real tension = SimTK::Pi;
        controller->addActuator(*actu);
        controller->prescribeControlForActuator(
                "path_actuator", Constant(tension));
        model.finalizeConnections();
        model.realizeDynamics(state);

        // Subtract out the gravitational forces to get only the forces due to
        // the cable tension.
        SimTK::Vector_<SimTK::SpatialVec> bodyForces =
                model.getRigidBodyForces(state) -
                model.getGravityBodyForces(state);
        for (int i = 0; i < bodyForces.size(); ++i) {
            CHECK_THAT((expectedBodyForces[i] * tension - bodyForces[i]).norm(),
                Catch::Matchers::WithinAbs(0., 1e-5));
        }
    }
}


// Simulate a Scholz2015GeometryPath over all supported surfaces and a via
// point, testing that we get the correct path kinematics. The path wraps over
// the following obstacles, in order: torus, ellipsoid, via point, sphere,
// cylinder, torus.
//
// This test is based on "testAllSurfaceKinds" in Simbody's TestCableSpan.cpp.
TEST_CASE("Path with all surfaces and a via point") {
    // Create an empty model.
    Model model;

    // Origin body and joint.
    auto* originBody = new Body("origin_body", 1.0, SimTK::Vec3(0),
        SimTK::Inertia(1.0));
    model.addComponent(originBody);
    auto* originJoint = new FreeJoint("origin_joint",
        model.getGround(), SimTK::Vec3(-8., 0.1, 0.), SimTK::Vec3(0),
        *originBody, SimTK::Vec3(0), SimTK::Vec3(0));
    model.addComponent(originJoint);

    // Via point body and joint.
    auto* viaPointBody = new Body("via_point_body", 1.0, SimTK::Vec3(0),
        SimTK::Inertia(1.0));
    model.addComponent(viaPointBody);
    auto* viaPointJoint = new FreeJoint("via_point_joint",
        model.getGround(), SimTK::Vec3(0., 0.9, 0.5), SimTK::Vec3(0),
        *viaPointBody, SimTK::Vec3(0), SimTK::Vec3(0));
    model.addComponent(viaPointJoint);

    // Insertion body and joint.
    auto* insertionBody = new Body("insertion_body", 1.0, SimTK::Vec3(0),
        SimTK::Inertia(1.0));
    model.addComponent(insertionBody);
    auto* insertionJoint = new FreeJoint("insertion_joint",
        model.getGround(), SimTK::Vec3(20., 1.0, -1.), SimTK::Vec3(0),
        *insertionBody, SimTK::Vec3(0), SimTK::Vec3(0));
    model.addComponent(insertionJoint);

    // Create the path object.
    Scholz2015GeometryPath* path = new Scholz2015GeometryPath();
    path->appendPathPoint(*originBody, SimTK::Vec3(0.));

    // Add the first torus obstacle.
    auto* torus1 = new ContactTorus(1., 0.2,
        SimTK::Vec3(-4., 0., 0.), SimTK::Vec3(0., 0.5*SimTK::Pi, 0.),
        model.getGround());
    torus1->setName("torus1");
    model.addComponent(torus1);
    path->appendObstacle(*torus1, SimTK::Vec3(0.1, 0.2, 0.));

    // Add the ellipsoid obstacle.
    auto* ellipsoid = new ContactEllipsoid(SimTK::Vec3(1.5, 2.6, 1.),
        SimTK::Vec3(-2., 0., 0.), SimTK::Vec3(0), model.getGround());
    ellipsoid->setName("ellipsoid");
    model.addComponent(ellipsoid);
    path->appendObstacle(*ellipsoid, SimTK::Vec3(0., 0., 1.1));

    // Add the via point.
    path->appendPathPoint(*viaPointBody, SimTK::Vec3(0.));

    // Add the sphere obstacle.
    auto* sphere = new ContactSphere(1.,
        SimTK::Vec3(2., 0., 0.), model.getGround());
    sphere->setName("sphere");
    model.addComponent(sphere);
    path->appendObstacle(*sphere, SimTK::Vec3(0.1, 1.1, 0.));

    // Add the cylinder obstacle.
    auto* cylinder = new ContactCylinder(1.,
        SimTK::Vec3(5., 0., 0.), SimTK::Vec3(0.5*SimTK::Pi, 0., 0.),
        model.getGround());
    cylinder->setName("cylinder");
    model.addComponent(cylinder);
    path->appendObstacle(*cylinder, SimTK::Vec3(0., -1., 0.));

    // Add the second torus obstacle.
    auto* torus2 = new ContactTorus(1., 0.2,
        SimTK::Vec3(14., 0., 0.), SimTK::Vec3(0., 0.5*SimTK::Pi, 0.),
        model.getGround());
    torus2->setName("torus2");
    model.addComponent(torus2);
    path->appendObstacle(*torus2, SimTK::Vec3(0.1, 0.2, 0.));

    // Add the insertion path point.
    path->appendPathPoint(*insertionBody, SimTK::Vec3(0.));

    // Add the path to the model.
    model.addComponent(path);

    // Initialize the system and state.
    SimTK::State state = model.initSystem();
    model.realizePosition(state);

    // Use this to assert cable length time derivative.
    SimTK::Real prevCableLength = SimTK::NaN;

    auto setJointKinematics =
            [&](FreeJoint& joint, SimTK::Vec3 q, SimTK::Vec3 u)
    {
        auto& x_coord = joint.updCoordinate(FreeJoint::Coord::TranslationX);
        x_coord.setValue(state, q[0]);
        x_coord.setSpeedValue(state, u[0]);

        auto& y_coord = joint.updCoordinate(FreeJoint::Coord::TranslationY);
        y_coord.setValue(state, q[1]);
        y_coord.setSpeedValue(state, u[1]);

        auto& z_coord = joint.updCoordinate(FreeJoint::Coord::TranslationZ);
        z_coord.setValue(state, q[2]);
        z_coord.setSpeedValue(state, u[2]);
    };

    // Let the path end and via points be parameterized by an angle, and draw
    // the path for different angles.
    const SimTK::Real dAngle = 1e-4;
    const SimTK::Real finalAngle = 0.5 * SimTK::Pi;
    for (SimTK::Real angle = 0.; angle < finalAngle; angle += dAngle) {

        // Move the cable origin.
        setJointKinematics(*originJoint,
                SimTK::Vec3(1.1 * sin(angle),
                            5. * sin(angle * 1.5),
                            5. * sin(angle * 2.)),
                SimTK::Vec3(1.1 * cos(angle),
                            5. * 1.5 * cos(angle * 1.5),
                            5. * 2. * cos(angle * 2.)));

        // Move the via point.
        setJointKinematics(*viaPointJoint,
                SimTK::Vec3(0., 0.5 * cos(angle), 0.),
                SimTK::Vec3(0., 0.5 * -sin(angle), 0.));

        // Move the cable insertion.
        setJointKinematics(*insertionJoint,
                SimTK::Vec3(0.1 * sin(angle),
                            4. * sin(angle * 0.7),
                            10. * sin(angle * 1.3)),
                SimTK::Vec3(0.1 * cos(angle),
                            4. * 0.7 * cos(angle * 0.7),
                            10. * 1.3 * cos(angle * 1.3)));

        // Compute the path.
        model.realizeVelocity(state);
        const SimTK::Real cableLength = path->getLength(state);

        // Assert length derivative using the change in length.
        if (!SimTK::isNaN(prevCableLength)) {
            const SimTK::Real tolerance = 5e-3;
            const SimTK::Real expectedCableLengtheningSpeed =
                    (cableLength - prevCableLength) / dAngle;
            const SimTK::Real gotCableLengtheningSpeed =
                    path->getLengtheningSpeed(state);
            CHECK_THAT(gotCableLengtheningSpeed,
                Catch::Matchers::WithinAbs(expectedCableLengtheningSpeed,
                    tolerance));
        }

        // Total path length should be longer than direct distance between the
        // path endpoints.
        const SimTK::Real distanceBetweenEndPoints =
            (insertionBody->getPositionInGround(state) -
             originBody->getPositionInGround(state))
                .norm();
        CHECK(cableLength > distanceBetweenEndPoints);

        prevCableLength = cableLength;
    }
}

// Simulate touchdown and liftoff events in a Scholz2015GeometryPath. A simple
// touchdown and liftoff case on torus, ellipsoid, sphere, and cylinder wrapping
// obstacles. A path is spanned over each obstacle individually, so there are 4
// paths, each with one obstacle.
//
// This test is based on "testTouchdownAndLiftoff" in Simbody's
// TestCableSpan.cpp.
TEST_CASE("Touchdown and liftoff") {
    // Create an empty model.
    Model model;

    // Helper for creating a path with a single obstacle at a certain offset
    // location.
    SimTK::Vec3 originShift(-2., 0., 0.);
    SimTK::Vec3 terminationShift(2., 0., 0.);
    auto createPath = [&](
        const std::string& name, const ContactGeometry& obstacle,
        const SimTK::Vec3& sceneOffset) {

        auto* originBody = new Body(
            fmt::format("{}_origin_body", name), 1.0, SimTK::Vec3(0),
            SimTK::Inertia(1.0));
        model.addComponent(originBody);
        auto* originJoint = new FreeJoint(fmt::format("{}_origin_joint", name),
            model.getGround(), sceneOffset + originShift, SimTK::Vec3(0),
            *originBody, SimTK::Vec3(0), SimTK::Vec3(0));
        model.addComponent(originJoint);

        // Termination body and joint.
        auto* terminationBody = new Body(
            fmt::format("{}_termination_body", name), 1.0, SimTK::Vec3(0),
            SimTK::Inertia(1.0));
        model.addComponent(terminationBody);
        auto* terminationJoint = new FreeJoint(
            fmt::format("{}_termination_joint", name),
            model.getGround(), sceneOffset + terminationShift, SimTK::Vec3(0),
            *terminationBody, SimTK::Vec3(0), SimTK::Vec3(0));
        model.addComponent(terminationJoint);

        // Create the path object.
        Scholz2015GeometryPath* path = new Scholz2015GeometryPath();
        path->setName(fmt::format("{}_path", name));
        path->appendPathPoint(*originBody, SimTK::Vec3(0.));
        path->appendObstacle(obstacle, SimTK::Vec3(SimTK::NaN));
        path->appendPathPoint(*terminationBody, SimTK::Vec3(0.));

        // Add the path to the model.
        model.addComponent(path);
    };

    // Create a path with a torus obstacle.
    SimTK::Vec3 sceneOffset(0., 2., 0.);
    auto* torus = new ContactTorus(2., 0.25,
        SimTK::Vec3(0., 1.75, 0.) + sceneOffset,
        SimTK::Vec3(0., 0.5*SimTK::Pi, 0.),
        model.getGround());
    torus->setName("torus");
    model.addComponent(torus);
    createPath("torus", *torus, sceneOffset);

    // Create a path with an ellipsoid obstacle.
    sceneOffset = SimTK::Vec3(0., 0., 0.);
    auto* ellipsoid = new ContactEllipsoid(SimTK::Vec3(1., 0.5, 0.75),
        SimTK::Vec3(0., -0.5, 0.) + sceneOffset,
        SimTK::Vec3(0., 0., 0.),
        model.getGround());
    ellipsoid->setName("ellipsoid");
    model.addComponent(ellipsoid);
    createPath("ellipsoid", *ellipsoid, sceneOffset);

    // Create a path with a sphere obstacle.
    sceneOffset = SimTK::Vec3(0., -2., 0.);
    auto* sphere = new ContactSphere(1.5,
        SimTK::Vec3(0., -1.5, 0.) + sceneOffset,
        model.getGround());
    sphere->setName("sphere");
    model.addComponent(sphere);
    createPath("sphere", *sphere, sceneOffset);

    // Create a path with a cylinder obstacle.
    sceneOffset = SimTK::Vec3(0., -6., 0.);
    auto* cylinder = new ContactCylinder(2.,
        SimTK::Vec3(0., -2., 0.) + sceneOffset,
        SimTK::Vec3(0),
        model.getGround());
    cylinder->setName("cylinder");
    model.addComponent(cylinder);
    createPath("cylinder", *cylinder, sceneOffset);

    // Initialize the system and state.
    SimTK::State state = model.initSystem();
    const SimTK::SimbodyMatterSubsystem& matter = model.getMatterSubsystem();
    const SimTK::CableSubsystem& cables = model.getCableSubsystem();
    CHECK(cables.getNumCables() == 4);

    // Simulate touchdown and liftoff events.
    for (SimTK::Real angle = 1e-2; angle < 4. * SimTK::Pi; angle += 0.02) {
        // Move the cable end points.
        const SimTK::Real yCoord = 0.1 * sin(angle);
        for (SimTK::CableSpanIndex cableIx(0); cableIx < cables.getNumCables();
             ++cableIx) {
            matter
                .getMobilizedBody(cables.getCable(cableIx).getOriginBodyIndex())
                .setQToFitTranslation(state, SimTK::Vec3(0., yCoord, 0.));
            matter
                .getMobilizedBody(
                    cables.getCable(cableIx).getTerminationBodyIndex())
                .setQToFitTranslation(state, SimTK::Vec3(0., yCoord, 0.));
        }

        model.realizePosition(state);

        // All obstacles are positioned such that for negative yCoord of the
        // endpoints, the cable touches down on the obstacle.
        for (SimTK::CableSpanIndex cableIx(0); cableIx < cables.getNumCables();
             ++cableIx) {
            cables.getCable(cableIx).calcLength(state);
            const bool gotContactStatus =
                cables.getCable(cableIx).isInContactWithObstacle(
                    state,
                    SimTK::CableSpanObstacleIndex(0));
            const bool expectedContactStatus = yCoord < 0.;
            CHECK(gotContactStatus == expectedContactStatus);
        }
    }
}

TEST_CASE("Path tension") {
    Model model = ModelFactory::createDoublePendulum();

    Scholz2015GeometryPath* path = new Scholz2015GeometryPath();
    path->setName("path");
    model.addComponent(path);
    path->appendPathPoint(model.getGround(), SimTK::Vec3(0.05, 0.05, 0.));
    path->appendPathPoint(model.getComponent<Body>("/bodyset/b0"),
            SimTK::Vec3(-0.75, 0.1, 0.));

    SimTK::State state = model.initSystem();
    model.realizePosition(state);

    SimTK::Vector_<SimTK::SpatialVec> bodyForces(3, SimTK::SpatialVec(0));
    SimTK::Vector mobilityForces(3, 0.0);

    SECTION("Unit tension") {
        const SimTK::Real tension = 1.0;
        path->addInEquivalentForces(state, tension, bodyForces, mobilityForces);
        CHECK(bodyForces[0].norm() > 0.0);
        CHECK(bodyForces[1].norm() > 0.0);
        CHECK(bodyForces[2].norm() == 0.0);
        CHECK(mobilityForces.norm() == 0.0);
    }

    SECTION("Zero tension") {
        const SimTK::Real tension = 0.0;
        path->addInEquivalentForces(state, tension, bodyForces, mobilityForces);
        for (int i = 0; i < bodyForces.size(); ++i) {
            CHECK(bodyForces[i].norm() == 0.0);
        }
        CHECK(mobilityForces.norm() == 0.0);
    }

    SECTION("Negative tension") {
        const SimTK::Real tension = -1.0;
        path->addInEquivalentForces(state, tension, bodyForces, mobilityForces);
        for (int i = 0; i < bodyForces.size(); ++i) {
            CHECK(bodyForces[i].norm() == 0.0);
        }
        CHECK(mobilityForces.norm() == 0.0);
    }

    SECTION("NaN tension") {
        const SimTK::Real tension = SimTK::NaN;
        path->addInEquivalentForces(state, tension, bodyForces, mobilityForces);
        CHECK(SimTK::isNaN(bodyForces[0].norm()));
        CHECK(SimTK::isNaN(bodyForces[1].norm()));
        CHECK(!SimTK::isNaN(bodyForces[2].norm()));
        CHECK(mobilityForces.norm() == 0.0);
    }
}

TEST_CASE("Changing contact geometry properties") {

    // Create a double pendulum model with two paths that wrap around the same
    // obstacle.
    Model model = ModelFactory::createDoublePendulum();
    Scholz2015GeometryPath* path1 = new Scholz2015GeometryPath();
    Scholz2015GeometryPath* path2 = new Scholz2015GeometryPath();
    path1->setName("path1");
    path2->setName("path2");
    model.addComponent(path1);
    model.addComponent(path2);

    // First path points.
    path1->appendPathPoint(model.getGround(), SimTK::Vec3(0.05, 0.05, 0.));
    path2->appendPathPoint(model.getGround(), SimTK::Vec3(0.05, 0.05, 0.));
    path1->appendPathPoint(model.getComponent<Body>("/bodyset/b0"),
            SimTK::Vec3(-0.75, 0.1, 0.));
    path2->appendPathPoint(model.getComponent<Body>("/bodyset/b0"),
            SimTK::Vec3(-0.75, 0.1, 0.));

    // Obstacle.
    auto* obstacle = new ContactCylinder(0.1,
        SimTK::Vec3(-0.5, 0.1, 0.), SimTK::Vec3(0),
        model.getComponent<Body>("/bodyset/b0"));
    obstacle->setName("cylinder");
    model.addComponent(obstacle);
    path1->appendObstacle(*obstacle, SimTK::Vec3(0., 0.1, 0.));
    path2->appendObstacle(*obstacle, SimTK::Vec3(0., 0.1, 0.));

    // Final path point.
    path1->appendPathPoint(model.getComponent<Body>("/bodyset/b1"),
            SimTK::Vec3(-0.25, 0.1, 0.));
    path2->appendPathPoint(model.getComponent<Body>("/bodyset/b1"),
            SimTK::Vec3(-0.25, 0.1, 0.));

    // Compute the path lengths.
    SimTK::State state = model.initSystem();
    model.realizePosition(state);
    SimTK::Real length1 = path1->getLength(state);
    SimTK::Real length2 = path2->getLength(state);
    // Sanity check.
    CHECK(length1 == length2);

    // Change the obstacle radius.
    model.updComponent<ContactCylinder>("/cylinder").setRadius(0.2);

    // We need to reinitialize the system *and* create a new state in order for
    // the radius change to take effect. This ensures that we do not access
    // cached path length values from the previous state.
    state = model.initSystem();

    // Calculate the new path lengths.
    model.realizePosition(state);
    SimTK::Real newLength1 = path1->getLength(state);
    SimTK::Real newLength2 = path2->getLength(state);

    // Check that the path lengths are longer than the original lengths.
    CHECK(newLength1 > length1);
    CHECK(newLength2 > length2);
}
