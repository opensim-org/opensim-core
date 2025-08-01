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
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Model/ContactGeometry.h>
#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Common/Sine.h>

using namespace OpenSim;

class DiscreteController : public Controller {
    OpenSim_DECLARE_CONCRETE_OBJECT(DiscreteController, Controller);
public:
    DiscreteController() = default;
    void setDiscreteControls(SimTK::State& s,
            const SimTK::Vector& controls) const;
    SimTK::Vector& updDiscreteControls(SimTK::State& s) const;
    const SimTK::Vector& getDiscreteControls(const SimTK::State& s) const;
    void computeControls(
            const SimTK::State& s, SimTK::Vector& controls) const override;
protected:
    void extendRealizeTopology(SimTK::State&) const override;
    mutable SimTK::DiscreteVariableIndex m_discreteVarIndex;

};

void DiscreteController::setDiscreteControls(SimTK::State& s,
        const SimTK::Vector& controls) const {
    updDiscreteControls(s) = controls;
}

SimTK::Vector& DiscreteController::updDiscreteControls(SimTK::State& s) const {
    const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
    auto& dv = subSys.updDiscreteVariable(s, m_discreteVarIndex);
    auto& discreteControls = SimTK::Value<SimTK::Vector>::updDowncast(dv).upd();
    return discreteControls;
}

const SimTK::Vector& DiscreteController::getDiscreteControls(
        const SimTK::State& s) const {
    const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
    auto& dv = subSys.getDiscreteVariable(s, m_discreteVarIndex);
    auto& discreteControls = SimTK::Value<SimTK::Vector>::downcast(dv).get();
    return discreteControls;
}

void DiscreteController::computeControls(
        const SimTK::State& s, SimTK::Vector& controls) const {
    const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
    const auto& dv = subSys.getDiscreteVariable(s, m_discreteVarIndex) ;
    const auto& discreteControls =
            SimTK::Value<SimTK::Vector>::downcast(dv).get();
    controls += discreteControls;
}

void DiscreteController::extendRealizeTopology(SimTK::State& state) const {
    Super::extendRealizeTopology(state);
    const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
    m_discreteVarIndex =
            subSys.allocateDiscreteVariable(state, SimTK::Stage::Dynamics,
                    new SimTK::Value<SimTK::Vector>(
                            SimTK::Vector(getModel().getNumControls(), 0.0)));
}

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

Model createNLinkPendulum(int numLinks) {
    Model model;
    OPENSIM_THROW_IF(numLinks < 0, Exception, "numLinks must be nonnegative.");
    std::string name;
    if (numLinks == 0) {
        name = "empty_model";
    } else if (numLinks == 1) {
        name = "pendulum";
    } else if (numLinks == 2) {
        name = "double_pendulum";
    } else {
        name = std::to_string(numLinks) + "_link_pendulum";
    }
    model.setName(name);
    const auto& ground = model.getGround();

    using SimTK::Inertia;
    using SimTK::Vec3;

    auto* root = new OpenSim::Body("root", 1, Vec3(0), Inertia(1));
    model.addBody(root);

    // Assume each body is 1 m long.
    auto* free = new FreeJoint("free", ground, Vec3(0), Vec3(0), *root,
            Vec3(0, 0, 0), Vec3(0));
    model.addJoint(free);

    Ellipsoid bodyGeometry(0.5, 0.1, 0.1);
    bodyGeometry.setColor(SimTK::Gray);

    const PhysicalFrame* prevBody = root;
    for (int i = 0; i < numLinks; ++i) {
        const std::string istr = std::to_string(i);
        auto* bi = new OpenSim::Body("b" + istr, 1, Vec3(0), Inertia(1));
        model.addBody(bi);

        // Assume each body is 1 m long.
        auto* ji = new PinJoint("j" + istr, *prevBody, Vec3(0), Vec3(0), *bi,
                Vec3(-1, 0, 0), Vec3(0));
        auto& qi = ji->updCoordinate();
        qi.setName("q" + istr);
        model.addJoint(ji);

        auto* taui = new CoordinateActuator();
        taui->setCoordinate(&ji->updCoordinate());
        taui->setName("tau" + istr);
        taui->setOptimalForce(1);
        model.addComponent(taui);

        auto* marker = new Marker("marker" + istr, *bi, Vec3(0));
        model.addMarker(marker);

        // Attach an ellipsoid to a frame located at the center of each body.
        PhysicalOffsetFrame* bicenter = new PhysicalOffsetFrame(
                "b" + istr + "center", *bi, SimTK::Transform(Vec3(-0.5, 0, 0)));
        bi->addComponent(bicenter);
        bicenter->attachGeometry(bodyGeometry.clone());

        prevBody = bi;
    }

    model.finalizeConnections();

    return model;
}

void simulateGeometryPath() {

    Model model = ModelFactory::createDoublePendulum();
    // model.updComponent<Coordinate>("/jointset/j0/q0").setDefaultValue(-1.0);
    // model.updComponent<Coordinate>("/jointset/j1/q1").setDefaultValue(0.25);
    model.setUseVisualizer(true);

    // Create a PathActuator with a GeometryPath.
    auto* actu = new PathActuator();
    actu->set_path(GeometryPath());
    model.addComponent(actu);   

    // Set the path's origin and insertion.
    GeometryPath& path = actu->updPath<GeometryPath>();
    path.appendNewPathPoint("origin", model.getGround(), SimTK::Vec3(0.25, 0, 0));
    path.appendNewPathPoint("insertion", model.getComponent<Body>("/bodyset/b1"), 
            SimTK::Vec3(-0.5, 0.1, 0));

    auto wrapFrame = new PhysicalOffsetFrame("patellaFrame",
        model.getComponent<Body>("/bodyset/b0"), SimTK::Transform(SimTK::Vec3(0, 0.05, 0)));
    // auto obstacle = new WrapEllipsoid();
    // obstacle->set_dimensions(SimTK::Vec3(0.2, 0.2, 0.5));
    // obstacle->set_quadrant("x");
    auto obstacle = new WrapCylinder();
    obstacle->set_radius(0.2);
    obstacle->set_length(0.5);
    obstacle->set_quadrant("x");

    wrapFrame->addWrapObject(obstacle);
    model.updComponent<Body>("/bodyset/b0").addComponent(wrapFrame);

    path.addPathWrap(*obstacle);


    SimTK::State state = model.initSystem();
    Manager manager(model);
    manager.setIntegratorMethod(Manager::IntegratorMethod::SemiExplicitEuler2);
    manager.initialize(state);

    auto start = std::chrono::high_resolution_clock::now();
    manager.integrate(10.0);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Integration took " << elapsed.count() << " seconds." << std::endl;
}

void simulateScholz2015GeometryPath() {

    Model model = createNLinkPendulum(2);
    // model.updComponent<Coordinate>("/jointset/j0/q0").setDefaultValue(-1.0);
    // model.updComponent<Coordinate>("/jointset/j1/q1").setDefaultValue(0.25);
    // model.setUseVisualizer(true);

    // Create a PathActuator with a Scholz2015GeometryPath.
    auto* actu = new PathActuator();
    actu->set_path(Scholz2015GeometryPath());
    model.addComponent(actu);   

    // Add a discrete controller to the model.
    DiscreteController* controller = new DiscreteController();
    controller->setName("controller");
    model.addController(controller);

    // Set the path's origin and insertion.
    Scholz2015GeometryPath& path = actu->updPath<Scholz2015GeometryPath>();
    path.setOrigin(model.getComponent<Body>("/bodyset/root"), SimTK::Vec3(0.25, 0, 0));
    path.setInsertion(model.getComponent<Body>("/bodyset/b1"), 
            SimTK::Vec3(-0.5, 0.1, 0));

    // auto* obstacle = new ContactEllipsoid(SimTK::Vec3(0.1, 0.1, 0.3),
        // SimTK::Vec3(0.5, -0.05, 0), SimTK::Vec3(0), model.getComponent<Body>("/bodyset/root"));
    auto* obstacle = new ContactCylinder(0.3,
        SimTK::Vec3(0.5, -0.05, 0), SimTK::Vec3(0), model.getComponent<Body>("/bodyset/root"));
    model.addComponent(obstacle);
    path.addObstacle(*obstacle, SimTK::Vec3(0.0, -0.3, 0.0));

    // Initialize the system.
    SimTK::State state = model.initSystem();
    SimTK::Vector controls(model.getNumControls(), 0.5);
    controller->setDiscreteControls(state, controls);

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

void simulateContactCylinder() {

    Model model = ModelFactory::createPendulum();
    model.setUseVisualizer(true);

    // Create a PathActuator with a Scholz2015GeometryPath.
    auto* actu = new PathSpring();
    actu->setName("path_spring");
    actu->setRestingLength(0.0);
    actu->setDissipation(5.0);
    actu->setStiffness(10.0);
    actu->set_path(Scholz2015GeometryPath());
    model.addComponent(actu);   

    // Set the path's origin and insertion.
    Scholz2015GeometryPath& path = actu->updPath<Scholz2015GeometryPath>();
    path.setOrigin(model.getGround(), SimTK::Vec3(-0.1, 0, 0));
    path.setInsertion(model.getComponent<Body>("/bodyset/b0"), 
            SimTK::Vec3(-0.5, 0.1, 0));

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

int main() {
    // simulateGeometryPath();
    // simulateScholz2015GeometryPath();
    simulateContactCylinder();
    return 0;
}

