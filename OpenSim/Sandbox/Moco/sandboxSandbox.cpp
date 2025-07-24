/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSandbox.cpp                                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

// This file provides a way to easily prototype or test temporary snippets of
// code during development.

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/Model/Scholz2015GeometryPath.h>
#include <OpenSim/Simulation/VisualizerUtilities.h>
#include <OpenSim/Simulation/Model/ContactGeometry.h>
#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Common/Sine.h>

using namespace OpenSim;

// int main() {
//     // Create a new model
//     Model model;
//     model.setName("BlockWithPath");

//     // Create first block body
//     double sphereMass = 1.0;
//     auto* block1 = new OpenSim::Body("block1", sphereMass, SimTK::Vec3(0),
//             sphereMass * SimTK::Inertia::sphere(0.05));
//     block1->attachGeometry(new Sphere(0.05));
//     model.addBody(block1);

//     // Create second block body
//     auto* block2 = new OpenSim::Body("block2", sphereMass, SimTK::Vec3(0),
//             sphereMass * SimTK::Inertia::sphere(0.05));
//     block2->attachGeometry(new Sphere(0.05));
//     model.addBody(block2);

//     // Add first block to model with slider joint
//     SimTK::Vec3 sliderOrientation(0, 0, SimTK::Pi/2.);
//     auto slider1ToGround = new SliderJoint("slider1", model.getGround(), SimTK::Vec3(0),
//                         sliderOrientation, *block1, SimTK::Vec3(0), sliderOrientation);
//     slider1ToGround->updCoordinate().setName("height1");
//     slider1ToGround->updCoordinate().setDefaultValue(1.0);
//     slider1ToGround->updCoordinate().set_prescribed(true);
//     const Sine& function = Sine(0.05, 2.0, 0.0, 0.55);
//     slider1ToGround->updCoordinate().setPrescribedFunction(function);
//     model.addJoint(slider1ToGround);

//     // Add second block with slider joint offset horizontally
//     sliderOrientation = SimTK::Vec3(0);
//     auto slider2ToGround = new SliderJoint("slider2", *block1, SimTK::Vec3(1.0, 0, 0),
//                         sliderOrientation, *block2, SimTK::Vec3(0), sliderOrientation);
//     slider2ToGround->updCoordinate().setName("height2");
//     slider2ToGround->updCoordinate().set_prescribed(true);
//     const Sine& function2 = Sine(0.05, 5.0, 0.2, 0.0);
//     slider2ToGround->updCoordinate().setPrescribedFunction(function2);
//     model.addJoint(slider2ToGround);

//     // Create wrap obstacles
//     auto* ellipsoid = new ContactEllipsoid(SimTK::Vec3(0.1, 0.1, 0.3),
//             SimTK::Vec3(0., 0.2, 0), SimTK::Vec3(0), model.getGround());
//     model.addComponent(ellipsoid);

//     auto* sphere = new ContactSphere(0.15, SimTK::Vec3(0.25, 0.6, 0),
//         model.getGround(), "wrap_sphere");
//     model.addComponent(sphere);

//     auto* cylinder = new ContactCylinder(0.1, SimTK::Vec3(0.75, 0.4, 0),
//             SimTK::Vec3(0.), model.getGround());
//     model.addComponent(cylinder);

//     auto* pathActuator = new PathActuator();
//     pathActuator->setName("path_actuator");
//     pathActuator->set_path(Scholz2015GeometryPath());
//     model.addComponent(pathActuator);

//     // Create and add geometry path with two segments
//     Scholz2015GeometryPath& path = pathActuator->updPath<Scholz2015GeometryPath>();
//     path.setOrigin(model.getGround(), SimTK::Vec3(0));
//     path.setInsertion(*block2, SimTK::Vec3(0));
//     path.setName("test_path");
//     path.addObstacle(*ellipsoid, SimTK::Vec3(0.1, 0., 0.));
//     path.addViaPoint(*block1, SimTK::Vec3(0));
//     path.addObstacle(*sphere, SimTK::Vec3(0., 0.5, 0.));
//     path.addObstacle(*cylinder, SimTK::Vec3(0., -0.1, 0.));

//     // Initialize system
//     SimTK::State state = model.initSystem();
//     model.print("Scholz2015GeometryPathModel.osim");

//     // VisualizerUtilities::showModel(model);

//     Manager manager(model);
//     manager.initialize(state);
//     manager.integrate(10.0);

//     // Visualize
//     TimeSeriesTable table = manager.getStatesTable();
//     VisualizerUtilities::showMotion(model, table);

//     return EXIT_SUCCESS;
// }

int main() {
    Model model = ModelFactory::createDoublePendulum();
    // model.setUseVisualizer(true);
    // model.updComponent<Coordinate>("/jointset/j0/q0").setDefaultValue(-1.2);
    // model.updComponent<Coordinate>("/jointset/j1/q1").setDefaultValue(0.4);

    // Create a PathActuator with a Scholz2015GeometryPath.
    // auto* actu = new PathActuator();
    // actu->set_path(Scholz2015GeometryPath());
    // model.addComponent(actu);   

    // Set the path's origin and insertion.
    // Scholz2015GeometryPath& path = actu->updPath<Scholz2015GeometryPath>();
    Scholz2015GeometryPath* path = new Scholz2015GeometryPath();
    path->setOrigin(model.getGround(), SimTK::Vec3(0.25, 0, 0));
    path->setInsertion(model.getComponent<Body>("/bodyset/b1"), 
            SimTK::Vec3(-0.5, 0.1, 0));

    auto* ellipsoid = new ContactEllipsoid(SimTK::Vec3(0.1, 0.1, 0.3),
        SimTK::Vec3(0., 0.2, 0), SimTK::Vec3(0), model.getComponent<Body>("/bodyset/b0"));
    model.addComponent(ellipsoid);
    path->addObstacle(*ellipsoid, SimTK::Vec3(0.5, 0.5, 0.));
    
    model.addComponent(path);

    // Add a via point.
    // path.addViaPoint(model.getComponent<Body>("/bodyset/b0"), 
    //         SimTK::Vec3(-0.5, 0.1, 0));



    SimTK::State state = model.initSystem();
    // VisualizerUtilities::showModel(model);

    Manager manager(model);
    manager.initialize(state);
    manager.integrate(10.0);

    // Visualize
    TimeSeriesTable table = manager.getStatesTable();
    VisualizerUtilities::showMotion(model, table);
}

