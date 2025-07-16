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
#include <OpenSim/Common/Sine.h>

using namespace OpenSim;

class ContactEllipsoid : public ContactGeometry {
OpenSim_DECLARE_CONCRETE_OBJECT(ContactEllipsoid, ContactGeometry);
public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(radii, SimTK::Vec3, "Radii of Ellipsoid."); 


    ContactEllipsoid(const SimTK::Vec3& radii, const SimTK::Vec3& location, 
        const SimTK::Vec3& orientation, const PhysicalFrame& frame) : 
        ContactGeometry(location, orientation, frame) {
        constructProperties();
        set_radii(radii);
    }

    SimTK::ContactGeometry createSimTKContactGeometry() const override {
        return SimTK::ContactGeometry::Ellipsoid(get_radii());
    }

    void constructProperties() {
        constructProperty_radii(SimTK::Vec3(0.5));
    }
};

class ContactCylinder : public ContactGeometry {
OpenSim_DECLARE_CONCRETE_OBJECT(ContactCylinder, ContactGeometry);
public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(radius, double, "Radius of the cylinder");
    OpenSim_DECLARE_PROPERTY(height, double, "Height of the cylinder");


    ContactCylinder(const double radius, const double height, const SimTK::Vec3& location, 
        const SimTK::Vec3& orientation, const PhysicalFrame& frame) : 
        ContactGeometry(location, orientation, frame) {
        constructProperties();
        set_radius(radius);
        set_height(height);
    }

    SimTK::ContactGeometry createSimTKContactGeometry() const override {
        return SimTK::ContactGeometry::Cylinder(get_radius());
    }

    void constructProperties() {
        constructProperty_radius(0.5);
        constructProperty_height(1.0);
    }
};


int main() {
    // Create a new model
    Model model;
    model.setName("BlockWithPath");

    // Create first block body
    double sphereMass = 1.0;
    auto* block1 = new OpenSim::Body("block1", sphereMass, SimTK::Vec3(0), 
            sphereMass * SimTK::Inertia::sphere(0.05));
    block1->attachGeometry(new Sphere(0.05));
    model.addBody(block1);

    // Create second block body
    auto* block2 = new OpenSim::Body("block2", sphereMass, SimTK::Vec3(0), 
            sphereMass * SimTK::Inertia::sphere(0.05));
    block2->attachGeometry(new Sphere(0.05));
    model.addBody(block2);

    // Add first block to model with slider joint
    SimTK::Vec3 sliderOrientation(0, 0, SimTK::Pi/2.);
    auto slider1ToGround = new SliderJoint("slider1", model.getGround(), SimTK::Vec3(0),
                        sliderOrientation, *block1, SimTK::Vec3(0), sliderOrientation);
    slider1ToGround->updCoordinate().setName("height1");
    slider1ToGround->updCoordinate().setDefaultValue(1.0);
    slider1ToGround->updCoordinate().set_prescribed(true);
    const Sine& function = Sine(0.05, 2.0, 0.0, 0.55);
    slider1ToGround->updCoordinate().setPrescribedFunction(function);
    model.addJoint(slider1ToGround);

    // Add second block with slider joint offset horizontally
    sliderOrientation = SimTK::Vec3(0);
    auto slider2ToGround = new SliderJoint("slider2", *block1, SimTK::Vec3(1.0, 0, 0),
                        sliderOrientation, *block2, SimTK::Vec3(0), sliderOrientation);
    slider2ToGround->updCoordinate().setName("height2"); 
    slider2ToGround->updCoordinate().set_prescribed(true);
    const Sine& function2 = Sine(0.05, 5.0, 0.2, 0.0);
    slider2ToGround->updCoordinate().setPrescribedFunction(function2);
    model.addJoint(slider2ToGround);

    // Create wrap obstacles
    auto* ellipsoid = new ContactEllipsoid(SimTK::Vec3(0.1, 0.1, 0.3), 
            SimTK::Vec3(0., 0.2, 0), SimTK::Vec3(0), model.getGround());
    model.addComponent(ellipsoid);

    auto* sphere = new ContactSphere(0.15, SimTK::Vec3(0.25, 0.6, 0), 
        model.getGround(), "wrap_sphere");
    model.addComponent(sphere);

    auto* cylinder = new ContactCylinder(0.1, 0.2, SimTK::Vec3(0.75, 0.4, 0), 
            SimTK::Vec3(0.), model.getGround());
    model.addComponent(cylinder);

    // Create stations for path endpoints and intermediate point
    auto* origin = new Station(model.getGround(), SimTK::Vec3(0));
    origin->setName("origin");
    model.addComponent(origin);

    auto* mid = new Station(*block1, SimTK::Vec3(0));
    mid->setName("mid");
    model.addComponent(mid);

    auto* insertion = new Station(*block2, SimTK::Vec3(0));
    insertion->setName("insertion");
    model.addComponent(insertion);

    // Create and add geometry path with two segments
    auto* path = new Scholz2015GeometryPath(*origin, *insertion);
    path->setName("test_path");
    path->addObstacle(*ellipsoid, SimTK::Vec3(0.1, 0., 0.));
    path->addViaPoint(*mid);
    path->addObstacle(*sphere, SimTK::Vec3(0., 0.5, 0.));
    path->addObstacle(*cylinder, SimTK::Vec3(0., -0.1, 0.));
    model.addComponent(path);

    // Initialize systemå
    SimTK::State state = model.initSystem();

    // VisualizerUtilities::showModel(model);

    Manager manager(model);
    manager.initialize(state);
    manager.integrate(10.0);

    // Visualize   
    TimeSeriesTable table = manager.getStatesTable();
    VisualizerUtilities::showMotion(model, table);

    return EXIT_SUCCESS;
}
