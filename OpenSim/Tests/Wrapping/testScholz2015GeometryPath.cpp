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

#include <OpenSim/Simulation/Model/Scholz2015GeometryPath.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Simulation/Model/PathSpring.h>
#include <OpenSim/Simulation/Manager/Manager.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;

TEST_CASE("Scholz2015GeometryPath interface") {
    Model model;
    Body* body = new OpenSim::Body("body", 1.0, SimTK::Vec3(0), 
        SimTK::Inertia(1.0));
    model.addBody(body);

    SliderJoint* slider = new SliderJoint("slider1", 
            model.getGround(), SimTK::Vec3(0),  SimTK::Vec3(0, 0, SimTK::Pi/2.), 
            *body, SimTK::Vec3(0), SimTK::Vec3(0, 0, SimTK::Pi/2.));
    slider->updCoordinate().setDefaultValue(1.0);
    model.addJoint(slider);

    auto* path = new Scholz2015GeometryPath(model.getGround(),
            SimTK::Vec3(0), *body, SimTK::Vec3(0));
    model.addComponent(path);

    SECTION("Origin is already connected") {
        CHECK_THROWS_AS(path->setOrigin(model.getGround(), SimTK::Vec3(0)),
            Exception);
    }

    SECTION("Insertion is already connected") {
        CHECK_THROWS_AS(path->setInsertion(*body, SimTK::Vec3(0)),
            Exception);
    }
}


TEST_CASE("Suspended pendulum") {

    // Create a single pendulum model.
    Model model = ModelFactory::createPendulum();

    // Add a path spring that will wrap over each ContactGeometry, suspending 
    // the pendulum. We use a relatively large dissipation coefficient to ensure
    // that the pendulum comes to rest quickly.
    auto* actu = new PathSpring();
    actu->setName("path_spring");
    actu->setRestingLength(0.0);
    actu->setDissipation(2.0);
    actu->setStiffness(50.0);
    actu->set_path(Scholz2015GeometryPath());
    model.addComponent(actu);   

    // Set the path's origin and insertion.
    Scholz2015GeometryPath& path = actu->updPath<Scholz2015GeometryPath>();
    path.setOrigin(model.getGround(), SimTK::Vec3(-0.1, 0, 0));
    path.setInsertion(model.getComponent<Body>("/bodyset/b0"), 
            SimTK::Vec3(-0.5, 0.1, 0));


    SECTION("ContactCylinder") {
        auto* obstacle = new ContactCylinder(0.1,
        SimTK::Vec3(0.25, 0, 0), SimTK::Vec3(0), model.getGround());
        model.addComponent(obstacle);
        path.addObstacle(*obstacle, SimTK::Vec3(0.0, 0.1, 0.0));

        // Initialize the system.
        SimTK::State state = model.initSystem();

        // Simulate.
        Manager manager(model);
        manager.setIntegratorMethod(Manager::IntegratorMethod::SemiExplicitEuler2);
        manager.initialize(state);

        auto start = std::chrono::high_resolution_clock::now();
        manager.integrate(10.0);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "Integration took " << elapsed.count() << " seconds." << std::endl;
    
        TimeSeriesTable table = manager.getStatesTable();
        VisualizerUtilities::showMotion(model, table);

    }
}