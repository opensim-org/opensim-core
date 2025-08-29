/* -------------------------------------------------------------------------- *
 *                     OpenSim:  testContactGeometry.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Peter Eastman, Ajay Seth, Nicholas Bianco                       *
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

#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/ForceReporter.h>
#include <OpenSim/Simulation/Model/ContactHalfSpace.h>
#include <OpenSim/Simulation/Model/ContactMesh.h>
#include <OpenSim/Simulation/Model/ElasticFoundationForce.h>
#include <OpenSim/Simulation/Model/HuntCrossleyForce.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;
namespace {
    const static double integ_accuracy = 1.0e-5;
    const static double duration = 1.0;
    const static double interval = 0.01;
    const static SimTK::Vec3 gravity_vec = SimTK::Vec3(0, -9.8065, 0);
    const static double mass = 1.0;
    const static double radius = 0.10;
    const static double height = 1.0;
    const static std::string mesh_files[] = {"sphere_10cm_radius.obj",
                                             "sphere_10cm_radius.stl",
                                             "sphere_10cm_radius.vtp"};

    // Test sphere-to-floor contact by simulating a "bouncing ball" model and
    // checking that energy is conserved.
    int testBouncingBall(bool useMesh, const std::string mesh_filename="") {

        // Instantiate a new OpenSim Model.
        Model* model = new Model;
        model->setName("TestContactGeometry_Ball");
        model->setGravity(gravity_vec);

        // Add a Body representing the ball.
        Body ball;
        ball.setName("ball");
        ball.set_mass(mass);
        ball.set_mass_center(SimTK::Vec3(0));
        ball.setInertia(SimTK::Inertia(1.0));

        // Add a FreeJoint between the ground and the ball.
        FreeJoint free("free",
                model->getGround(), SimTK::Vec3(0), SimTK::Vec3(0),
                ball, SimTK::Vec3(0), SimTK::Vec3(0));
        model->addBody(&ball);
        model->addJoint(&free);

        // Create the floor ContactGeometry.
        ContactHalfSpace* floor = new ContactHalfSpace(SimTK::Vec3(0),
                SimTK::Vec3(0, 0, -0.5*SimTK::Pi),
                model->getGround(), "floor");
        model->addContactGeometry(floor);

        // Create the ball ContactGeometry.
        ContactGeometry* geometry;
        if (useMesh){
            geometry = new ContactMesh(mesh_filename, SimTK::Vec3(0),
                    SimTK::Vec3(0), ball, "sphere");
        } else {
            geometry = new ContactSphere(radius, SimTK::Vec3(0), ball,
                    "sphere");
        }
        model->addContactGeometry(geometry);

        // Define the contact force model.
        Force* force;
        if (useMesh) {
            // Add an ElasticFoundationForce.
            auto* contactParams =
                new ElasticFoundationForce::ContactParameters(
                        1.0e6/radius, 1e-5, 0.0, 0.0, 0.0);
            contactParams->addGeometry("sphere");
            contactParams->addGeometry("floor");
            force = new ElasticFoundationForce(contactParams);
            model->addForce(force);
        } else {
            // Add a HuntCrossleyForce.
            auto* contactParams =
                new HuntCrossleyForce::ContactParameters(
                        1.0e6, 1e-5, 0.0, 0.0, 0.0);
            contactParams->addGeometry("sphere");
            contactParams->addGeometry("floor");
            force = new HuntCrossleyForce(contactParams);
            model->addForce(force);
        }

        // Print the model to a file.
        model->clone()->print("TestContactGeometry_Ball.osim");

        // Add a Kinematics analysis to the model.
        Kinematics* kin = new Kinematics(model);
        model->addAnalysis(kin);

        // Initialize the system and state, and set the initial height.
        SimTK::State state = model->initSystem();
        state.updQ()[4] = height;
        model->realizeDynamics(state);

        // The initial system energy is all potential energy.
        double initialPotentialEnergy = model->calcPotentialEnergy(state);
        double initialKineticEnergy = model->calcKineticEnergy(state);
        double initialTotalEnergy =
                initialPotentialEnergy + initialKineticEnergy;
        CHECK_THAT(initialKineticEnergy,
                Catch::Matchers::WithinAbs(0.0, 1e-8));

        // Simulate the system to see if the ball bounces correctly.
        Manager manager(*model);
        manager.setIntegratorAccuracy(integ_accuracy);
        state.setTime(0.0);
        manager.initialize(state);
        for (unsigned int i = 0; i < duration/interval; ++i) {
            state = manager.integrate((i + 1)*interval);
            double time = state.getTime();

            model->realizeAcceleration(state);
            SimTK::Vec3 origin(0);
            SimTK::Vec3 pos = ball.findStationLocationInGround(state, origin);
            SimTK::Vec3 vel = ball.findStationVelocityInGround(state, origin);

            double totalEnergy = model->calcPotentialEnergy(state) +
                    model->calcKineticEnergy(state);

            // The contact absorbs and returns energy, but only check when we
            // are not in contact.
            if (pos[1] > 2*radius) {
                CHECK_THAT(totalEnergy,
                    Catch::Matchers::WithinAbs(initialTotalEnergy, 1e-2));
            } else {
                INFO("In contact at time = " << time);
                CHECK(pos[1] < 5.0);
                CHECK(pos[1] > 0);
            }
            CHECK_THAT(pos[0], Catch::Matchers::WithinAbs(0.0, 1e-4));
            CHECK_THAT(pos[2], Catch::Matchers::WithinAbs(0.0, 1e-4));
            CHECK_THAT(vel[0], Catch::Matchers::WithinAbs(0.0, 1e-3));
            CHECK_THAT(vel[2], Catch::Matchers::WithinAbs(0.0, 1e-3));
        }

        // Write the kinematics to a file.
        std::string prefix = useMesh ? "Kinematics_Mesh" : "Kinematics_NoMesh";
        kin->printResults(prefix);

        // Disown all components and delete the model.
        model->disownAllComponents();
        delete model;

        return 0;
    }

    // Test sphere-to-sphere contact using elastic foundation (with and without
    // contact meshes) and Hunt-Crossley contact.
    int testBallToBallContact(bool useElasticFoundation,
            bool useFirstSphereMesh, bool useSecondSphereMesh) {

        // Instantiate a new OpenSim Model.
        Model* model = new Model;
        model->setGravity(gravity_vec);

        // Add a Body for the ball.
        Body ball;
        ball.setName("ball");
        ball.setMass(mass);
        ball.setMassCenter(SimTK::Vec3(0));
        ball.setInertia(SimTK::Inertia(1.0));

        // Add a FreeJoint between the ball and the ground.
        FreeJoint free("free",
                model->getGround(), SimTK::Vec3(0), SimTK::Vec3(0),
                ball, SimTK::Vec3(0), SimTK::Vec3(0));
        model->addBody(&ball);
        model->addJoint(&free);

        // Create the ContactGeometry attached to ground.
        ContactGeometry* ball1;
        if (useElasticFoundation && useFirstSphereMesh) {
            ball1 = new ContactMesh(mesh_files[0], SimTK::Vec3(0),
                    SimTK::Vec3(0), model->getGround(), "ball1");
        } else {
            ball1 = new ContactSphere(radius, SimTK::Vec3(0),
                    model->getGround(), "ball1");
        }

        // Create the ContactGeometry attached to the ball Body.
        ContactGeometry* ball2;
        if (useElasticFoundation && useSecondSphereMesh) {
            ball2 = new ContactMesh(mesh_files[0], SimTK::Vec3(0),
                    SimTK::Vec3(0), ball, "ball2");
        } else {
            ball2 = new ContactSphere(radius, SimTK::Vec3(0), ball, "ball2");
        }
        model->addContactGeometry(ball1);
        model->addContactGeometry(ball2);

        // Create the contact force.
        Force* force;
        std::string prefix;
        if (useElasticFoundation) {
            // Add an ElasticFoundationForce.
            auto* contactParams =
                new ElasticFoundationForce::ContactParameters(
                        1.0e6/(2*radius), 0.001, 0.0, 0.0, 0.0);
            contactParams->addGeometry("ball1");
            contactParams->addGeometry("ball2");
            force = new ElasticFoundationForce(contactParams);
            prefix = "EF_";
            prefix += useFirstSphereMesh ? "Mesh" : "noMesh";
            prefix += useSecondSphereMesh ? "_to_Mesh" : "_to_noMesh";

        } else {
            // Add a Hertz HuntCrossleyForce.
            auto* contactParams =
                new HuntCrossleyForce::ContactParameters(
                        1.0e6, 0.001, 0.0, 0.0, 0.0);
            contactParams->addGeometry("ball1");
            contactParams->addGeometry("ball2");
            force = new HuntCrossleyForce(contactParams);
            prefix = "Hertz";
        }
        force->setName("contact");
        model->addForce(force);

        // Set the model name and print it.
        model->setName(prefix);
        model->clone()->print(prefix+".osim");

        // Create the kinematics and force reporter analyses.
        Kinematics* kin = new Kinematics(model);
        model->addAnalysis(kin);
        ForceReporter* reporter = new ForceReporter(model);
        model->addAnalysis(reporter);

        // Initialize the system and set the initial height.
        SimTK::State state = model->initSystem();
        state.updQ()[4] = height;
        model->realizePosition(state);

        // Simulate the system to see if the ball bounces correctly.
        Manager manager(*model);
        manager.setIntegratorAccuracy(integ_accuracy);
        manager.setIntegratorMaximumStepSize(100*integ_accuracy);
        state.setTime(0.0);
        manager.initialize(state);
        state = manager.integrate(duration);

        // Print the Analysis results.
        kin->printResults(prefix);
        reporter->printResults(prefix);

        // Disown all components and delete the model.
        model->disownAllComponents();
        delete model;

        return 0;
    }

    // In version 4.0, we introduced intermediate PhysicalFrames to
    // ContactGeometry. The test below ensures that the intermediate frames (as
    // well as the ContactGeometry's location and orientation properties) are
    // accounted for by comparing results for equivalent systems, some of which
    // use an intermediate offset frame.
    // The system is a point mass situated 1 meter along a link that is attached
    // to ground by a hinge. The contact ball (sphere or mesh) is 1 meter up and
    // 0.5 meters to the right of the mass (in the frame of the link). The ball
    // bounces on a platform, whose orientation is horizontal (or, 90 degrees
    // clockwise from its "default" orientation, which is vertical).
    //
    //                                       0.5 m
    //                                     -----(ball)
    //                                     |
    //                                     |
    //                                     | 1 m
    //                             1 m     |
    //          ________(hinge)----------(mass)_______________________
    //                     ^                      (platform half-space)
    //                     |                       must be rotated 90 deg.
    //                  origin                     clockwise to achieve the
    //                                             horizontal orientation
    //
    // The link starts at an incline of 27 degrees and then the link drops down
    // and hits the platform.
    // We test three equivalent systems that specify the transforms for the
    // platform and ball geometries in different ways:
    //    1. Only using WeldJoints and massless bodies.
    //    2. Using a mix of PhysicalOffsetFrames and the geometry's location and
    //       orientation properties.
    //    3. Only using PhysicalOffsetFrames.

    // Add contact geometry and contact force components to the model.
    template <typename ContactType>
    void addContactComponents(Model& model,
            PhysicalFrame& frameForBall, SimTK::Vec3 locForBall,
            PhysicalFrame& frameForPlatform,
            SimTK::Vec3 orientationForPlatform);

    // Specialize for Hunt-Crossley.
    template<>
    void addContactComponents<HuntCrossleyForce>(Model& model,
            PhysicalFrame& frameForBall, SimTK::Vec3 locForBall,
            PhysicalFrame& frameForPlatform,
            SimTK::Vec3 orientationForPlatform) {

        // ContactGeometry.
        // By default, x > 0 is inside the half space.
        auto* floor = new ContactHalfSpace(SimTK::Vec3(0),
                orientationForPlatform, frameForPlatform, "platform");
        auto* geometry = new ContactSphere(radius, locForBall, frameForBall,
                "ball");
        model.addContactGeometry(floor);
        model.addContactGeometry(geometry);

        // Force.
        auto* contactParams = new HuntCrossleyForce::ContactParameters(
                1.0e6, 1e-5, 0.0, 0.0, 0.0);
        contactParams->addGeometry("ball");
        contactParams->addGeometry("platform");
        auto* force = new HuntCrossleyForce(contactParams);
        model.addForce(force);
    }

    // Specialize for Elastic Foundation.
    template<>
    void addContactComponents<ElasticFoundationForce>(Model& model,
            PhysicalFrame& frameForBall, SimTK::Vec3 locForBall,
            PhysicalFrame& frameForPlatform,
            SimTK::Vec3 orientationForPlatform) {

        // ContactGeometry.
        // By default, x > 0 is inside the half space.
        auto* floor = new ContactHalfSpace(SimTK::Vec3(0),
                orientationForPlatform, frameForPlatform, "platform");
        auto* geometry = new ContactMesh(mesh_files[0], locForBall,
                SimTK::Vec3(0), frameForBall, "ball");
        model.addContactGeometry(floor);
        model.addContactGeometry(geometry);

        // Force.
        auto* contactParams = new ElasticFoundationForce::ContactParameters(
                1.0e6, 1e-5, 0.0, 0.0, 0.0);
        contactParams->addGeometry("ball");
        contactParams->addGeometry("platform");
        auto* force = new ElasticFoundationForce(contactParams);
        model.addForce(force);
    }

    Model createBaseModel() {
        Model model;
        // For debugging: model.setUseVisualizer(true);

        // Body and Joint.
        auto* point = new Body("point", mass, SimTK::Vec3(0),
                SimTK::Inertia(1.0));
        auto* hinge = new PinJoint("hinge",
                model.getGround(), SimTK::Vec3(0), SimTK::Vec3(0),
                *point, SimTK::Vec3(-1, 0, 0), SimTK::Vec3(0));

        model.addBody(point);
        model.addJoint(hinge);

        return model;
    }

    template <typename ContactType>
    void testIntermediateFrames() {

        // Simulate the given model for one second and return the final state.
        auto simulate = [](Model& model) -> SimTK::State {
            // Initialize and set state.
            SimTK::State& state = model.initSystem();
            const auto& hinge = model.getJointSet().get("hinge");
            const auto& coord = hinge.getCoordinate();
            coord.setValue(state, 0.15 * SimTK::Pi);

            // Integrate.
            Manager manager(model);
            manager.setIntegratorAccuracy(integ_accuracy);
            state.setTime(0.0);
            manager.initialize(state);
            state = manager.integrate(1.0);

            return state;
        };

        const SimTK::Real deg90 = 0.5 * SimTK::Pi;
        const SimTK::Real deg45 = 0.25 * SimTK::Pi;

        // Achieve transforms with weld joints and massless bodies.
        SimTK::State stateWeld;
        {
            Model model = createBaseModel();

            // Scaffolding for the ball.
            const auto& point = model.getBodySet().get("point");
            auto* linkOffset = new Body("link_offset", 0, SimTK::Vec3(0),
                    SimTK::Inertia(0.0));
            auto* linkWeld = new WeldJoint("link_weld",
                    point, SimTK::Vec3(0), SimTK::Vec3(0),
                    // Body is 0.5m to the right and 1m up.
                    *linkOffset, SimTK::Vec3(-0.5, -1, 0), SimTK::Vec3(0));
            model.addBody(linkOffset);
            model.addJoint(linkWeld);

            // Scaffolding for the platform.
            auto* platformOffset = new Body("platform_offset", 0,
                    SimTK::Vec3(0), SimTK::Inertia(0));
            auto* platformWeld = new WeldJoint("platform_weld",
                    model.getGround(), SimTK::Vec3(0), SimTK::Vec3(0),
                    *platformOffset, SimTK::Vec3(0), SimTK::Vec3(0));
            model.addBody(platformOffset);
            model.addJoint(platformWeld);

            addContactComponents<ContactType>(model,
                    *linkOffset, SimTK::Vec3(0),
                    *platformOffset, SimTK::Vec3(0, 0, -deg90));
            stateWeld = simulate(model);

            // Make sure this model actually bounced; otherwise, the test is not
            // meaningful.
            SimTK_TEST(model.calcMassCenterVelocity(stateWeld)[1] > 0);
        }

        // Achieve transforms with a mix of PhysicalOffsetFrames and
        // ContactGeometry's location and orientation properties.
        SimTK::State stateIntermedFrameY;
        {
            Model model = createBaseModel();

            // Scaffolding for the ball.
            auto* linkOffset = new PhysicalOffsetFrame("link_offset",
                    model.getBodySet().get("point"),
                    // Frame is up 1m in the y direction.
                    SimTK::Transform(SimTK::Vec3(0, 1, 0)));
            model.addComponent(linkOffset);

            // Scaffolding for the platform.
            auto* platformOffset = new PhysicalOffsetFrame("platform_offset",
                    model.getGround(),
                    SimTK::Transform(SimTK::Rotation(-deg45, SimTK::ZAxis)));
            model.addComponent(platformOffset);

            addContactComponents<ContactType>(model,
                    *linkOffset, SimTK::Vec3(0.5, 0, 0),
                    *platformOffset, SimTK::Vec3(0, 0, -deg45));
            stateIntermedFrameY = simulate(model);

            // Make sure this model actually bounced; otherwise, the test is not
            // meaningful.
            SimTK_TEST(model.calcMassCenterVelocity(stateIntermedFrameY)[1] > 0);
        }

        // Achieve transforms solely with PhysicalOffsetFrames.
        SimTK::State stateIntermedFrameXY;
        {
            Model model = createBaseModel();

            // Scaffolding for the ball.
            auto* linkOffset = new PhysicalOffsetFrame("link_offset",
                    model.getBodySet().get("point"),
                    // Frame is 0.5m to the right and 1m up.
                    SimTK::Transform(SimTK::Vec3(0.5, 1, 0)));
            model.addComponent(linkOffset);

            // Scaffolding for the platform.
            auto* platformOffset = new PhysicalOffsetFrame("platform_offset",
                    model.getGround(),
                    SimTK::Transform(SimTK::Rotation(-deg90, SimTK::ZAxis)));
            model.addComponent(platformOffset);

            addContactComponents<ContactType>(model,
                    *linkOffset, SimTK::Vec3(0),
                    *platformOffset, SimTK::Vec3(0));
            stateIntermedFrameXY = simulate(model);

            // Make sure this model actually bounced; otherwise, the test is not
            // meaningful.
            SimTK_TEST(model.calcMassCenterVelocity(stateIntermedFrameXY)[1] > 0);
        }

        SimTK_TEST_EQ_TOL(stateWeld.getY(), stateIntermedFrameY.getY(), 1e-10);
        SimTK_TEST_EQ_TOL(stateWeld.getY(), stateIntermedFrameXY.getY(), 1e-10);
    }
}

TEST_CASE("Bouncing Ball") {
    SECTION("no mesh") {
        testBouncingBall(false);
    }
    SECTION("mesh .obj format") {
        testBouncingBall(true, mesh_files[0]);
    }
    SECTION("mesh .stl format") {
        testBouncingBall(true, mesh_files[1]);
    }
    SECTION("mesh .vtp format") {
        testBouncingBall(true, mesh_files[2]);
    }
}

TEST_CASE("Ball to Ball Contact") {
    testBallToBallContact(false, false, false);
    testBallToBallContact(true, true, false);
    testBallToBallContact(true, false, true);
    testBallToBallContact(true, true, true);
}

TEST_CASE("Compare Hertz and Mesh Contact Results") {
    Storage hertz("Hertz_ForceReporter_forces.sto");
    Storage meshToMesh("EF_Mesh_to_Mesh_ForceReporter_forces.sto");
    Storage noMeshToMesh("EF_noMesh_to_Mesh_ForceReporter_forces.sto");
    Storage meshToNoMesh("EF_Mesh_to_noMesh_ForceReporter_forces.sto");

    int nforces = hertz.getColumnLabels().getSize()-1;

    // Hertz theory and ElasticFoundation will not be the same, but they should
    // yield similar results, to withing
    std::vector<double> rms_tols_1(nforces, 12);
    CHECK_STORAGE_AGAINST_STANDARD(meshToMesh, hertz, rms_tols_1,
            __FILE__, __LINE__,
            "ElasticFoundation FAILED to Match Hertz Contact ");

    // ElasticFoundation on mesh to mesh and mesh to non-mesh should be
    // virtually identical
    std::vector<double> rms_tols_2(nforces, 0.5);
    CHECK_STORAGE_AGAINST_STANDARD(meshToMesh, meshToNoMesh, rms_tols_2,
            __FILE__, __LINE__,
            "ElasticFoundation Mesh-Mesh FAILED to match Mesh-noMesh Case ");

    // ElasticFoundation on non-mesh to mesh and mesh to non-mesh should be
    // identical
    std::vector<double> rms_tols_3(nforces, integ_accuracy);
    CHECK_STORAGE_AGAINST_STANDARD(noMeshToMesh, meshToNoMesh, rms_tols_3,
            __FILE__, __LINE__,
            "ElasticFoundation noMesh-Mesh FAILED to match Mesh-noMesh Case ");

}

TEST_CASE("Intermediate Frames") {
    testIntermediateFrames<HuntCrossleyForce>();
    testIntermediateFrames<ElasticFoundationForce>();
}

TEST_CASE("ContactEllipsoid") {

    // Create a model with a single body and a FreeJoint.
    Model model;
    Body* body = new Body("body", mass, SimTK::Vec3(0), SimTK::Inertia(1.0));
    FreeJoint* free = new FreeJoint("free",
            model.getGround(), SimTK::Vec3(0), SimTK::Vec3(0),
            *body, SimTK::Vec3(0), SimTK::Vec3(0));
    model.addBody(body);
    model.addJoint(free);

    // Add a ContactEllipsoid and ContactHalfSpace to the model.
    ContactEllipsoid* ellipsoid = new ContactEllipsoid(
            SimTK::Vec3(0.1, 0.05, 0.03), SimTK::Vec3(0), SimTK::Vec3(0),
            *body);
    ellipsoid->setName("ellipsoid");
    model.addContactGeometry(ellipsoid);

    auto* floor = new ContactHalfSpace(SimTK::Vec3(0),
            SimTK::Vec3(0, 0, -0.5 * SimTK::Pi), model.getGround(), "floor");
    model.addContactGeometry(floor);

    // Add a Hertz HuntCrossleyForce.
    auto* contactParams = new HuntCrossleyForce::ContactParameters(
            1.0e6, 1.0, 0.5, 0.5, 0.5);
    contactParams->addGeometry("ellipsoid");
    contactParams->addGeometry("floor");
    HuntCrossleyForce* force = new HuntCrossleyForce(contactParams);
    force->setName("contact");
    model.addForce(force);

    // Build the system and set the initial height.
    SimTK::State state = model.initSystem();
    state.updQ()[4] = 1.0;

    // Simulate the model for 2 seconds.
    Manager manager(model);
    state.setTime(0.0);
    manager.initialize(state);
    state = manager.integrate(5.0);

    // Check that the system is at rest.
    REQUIRE_THAT(state.getU()[4], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Serialization and deserialization") {
    // Create an empty model.
    Model model;

    // Add a ContactEllipsoid to the model.
    ContactEllipsoid* ellipsoid = new ContactEllipsoid(
            SimTK::Vec3(0.1, 0.05, 0.03), SimTK::Vec3(0), SimTK::Vec3(0),
            model.getGround(), "ellipsoid");
    ellipsoid->setName("ellipsoid");
    model.addContactGeometry(ellipsoid);

    // Add a ContactSphere to the model.
    ContactSphere* sphere = new ContactSphere(
            0.1, SimTK::Vec3(0), model.getGround(), "sphere");
    sphere->setName("sphere");
    model.addContactGeometry(sphere);

    // Add a ContactCylinder to the model.
    ContactCylinder* cylinder = new ContactCylinder(
            0.1, SimTK::Vec3(0), SimTK::Vec3(0), model.getGround(), "cylinder");
    cylinder->setName("cylinder");
    model.addContactGeometry(cylinder);

    // Add a ContactTorus to the model.
    ContactTorus* torus = new ContactTorus(0.1, 0.2, SimTK::Vec3(0),
            SimTK::Vec3(0), model.getGround(), "torus");
    torus->setName("torus");
    model.addContactGeometry(torus);

    // Add a ContactHalfSpace to the model.
    auto* floor = new ContactHalfSpace(SimTK::Vec3(0),
            SimTK::Vec3(0, 0, -0.5 * SimTK::Pi), model.getGround(), "floor");
    model.addContactGeometry(floor);

    // Build the system and set the initial height.
    model.initSystem();

    SECTION("Serialization") {
        model.print("testContactGeometry.osim");
    }

    SECTION("Deserialization") {
        Model model2("testContactGeometry.osim");
        CHECK(model == model2);
    }
}
