/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testREADME.cpp                                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2022 Stanford University and the Authors                *
 * Author(s): Chris Dembia                                                    *
 * Contributor(s): Thomas Uchida, James Dunne, Connor Rawlings, Samuel Suys   *
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

/* Whenever you change this test:
 * 1. Copy the new code over to the README.md, making sure to omit the
 * preprocessor lines (#ifdef VISUALIZE; #endif).
 *
 * If your changes would cause the gif to change substantially, make a new gif:
 * 1. Uncomment the `VISUALIZE` definition.
 * 2. Recompile and execute this code (testREADME in your build directory).
 * 3. When the visualizer pops up, click View -> Save Movie.
 * 4. cd into testREADME_1 and run the following commands (on Linux):
 *      $ convert 'Frame*.png[400x470+200+100]' \( +clone -set delay 100 \)
 *          +swap +delete opensim_double_pendulum_muscle_1.gif
 *      $ gifsicle --crop-transparency --optimize=O3 --colors=32 --delay 5 <
 *          opensim_double_pendulum_muscle_1.gif >
 *          opensim_double_pendulum_muscle.gif
 * 5. Copy your gif over to OpenSim/doc/images, and commit it to the
 *    repository.
 */

// #define VISUALIZE

/// [README]
#include <OpenSim/OpenSim.h>
using namespace SimTK;
using namespace OpenSim;

int main() {
    Model model;
    model.setName("bicep_curl");
#ifdef VISUALIZE
    model.setUseVisualizer(true);
#endif

    // Create two links, each with a mass of 1 kg, center of mass at the body's
    // origin, and moments and products of inertia corresponding to an
    // ellipsoid with radii of 0.1, 0.5 and 0.1, in the x, y and z directions,
    // respectively.
    OpenSim::Body* humerus = new OpenSim::Body(
        "humerus", 1, Vec3(0), Inertia(0.052, 0.004, 0.052));
    OpenSim::Body* radius  = new OpenSim::Body(
        "radius",  1, Vec3(0), Inertia(0.052, 0.004, 0.052));

    // Connect the bodies with pin joints. Assume each body is 1 m long.
    PinJoint* shoulder = new PinJoint("shoulder",
            // Parent body, location in parent, orientation in parent.
            model.getGround(), Vec3(0), Vec3(0),
            // Child body, location in child, orientation in child.
            *humerus, Vec3(0, 0.5, 0), Vec3(0));
    PinJoint* elbow = new PinJoint("elbow",
            *humerus, Vec3(0, -0.5, 0), Vec3(0),
            *radius, Vec3(0, 0.5, 0), Vec3(0));

    // Add a muscle that flexes the elbow.
    Millard2012EquilibriumMuscle* biceps = new
        Millard2012EquilibriumMuscle("biceps", 100, 0.6, 0.55, 0);
    biceps->addNewPathPoint("origin",    *humerus, Vec3(0, 0.3, 0));
    biceps->addNewPathPoint("insertion", *radius,  Vec3(0, 0.2, 0));

    // Add a controller that specifies the excitation of the muscle.
    PrescribedController* brain = new PrescribedController();
    brain->addActuator(*biceps);
    // Muscle excitation is 0.3 for the first 0.5 seconds, then increases to 1.
    brain->prescribeControlForActuator("biceps",
            new StepFunction(0.5, 3, 0.3, 1));

    // Add components to the model.
    model.addBody(humerus);    model.addBody(radius);
    model.addJoint(shoulder);  model.addJoint(elbow);
    model.addForce(biceps);
    model.addController(brain);

    // Add a console reporter to print the muscle fiber force and elbow angle.
    ConsoleReporter* reporter = new ConsoleReporter();
    reporter->set_report_time_interval(1.0);
    reporter->addToReport(biceps->getOutput("fiber_force"));
    reporter->addToReport(
        elbow->getCoordinate(PinJoint::Coord::RotationZ).getOutput("value"),
        "elbow_angle");
    model.addComponent(reporter);

    // Add display geometry.
    Ellipsoid bodyGeometry(0.1, 0.5, 0.1);
    bodyGeometry.setColor(Gray);
    // Attach an ellipsoid to a frame located at the center of each body.
    PhysicalOffsetFrame* humerusCenter = new PhysicalOffsetFrame(
        "humerusCenter", *humerus, Transform(Vec3(0)));
    humerus->addComponent(humerusCenter);
    humerusCenter->attachGeometry(bodyGeometry.clone());
    PhysicalOffsetFrame* radiusCenter = new PhysicalOffsetFrame(
        "radiusCenter", *radius, Transform(Vec3(0)));
    radius->addComponent(radiusCenter);
    radiusCenter->attachGeometry(bodyGeometry.clone());

    // Configure the model.
    State& state = model.initSystem();
    // Fix the shoulder at its default angle and begin with the elbow flexed.
    shoulder->getCoordinate().setLocked(state, true);
    elbow->getCoordinate().setValue(state, 0.5 * Pi);
    model.equilibrateMuscles(state);

    // Configure the visualizer.
#ifdef VISUALIZE
    model.updMatterSubsystem().setShowDefaultGeometry(true);
    Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundType(viz.SolidColor);
    viz.setBackgroundColor(White);
#endif

    // Simulate.
    simulate(model, state, 5.0);

    return 0;
};
/// [README]
