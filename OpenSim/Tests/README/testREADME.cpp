/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testREADME.cpp                                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
 * Author(s): Chris Dembia                                                    *
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
 * 1. Copy the new code over to the README.md, making the necessary changes
 *    related to the `TESTING` definition.
 *
 * If your changes would cause the gif to change substantially:
 * 1. Uncomment the `TESTING` defininition.
 * 2. When the visualizer pops up, click View -> Save Movie.
 * 3. cd into testREADME_1 and run the following commands (on Linux):
 *      $ convert 'Frame*.png[400x470+200+100]' \( +clone -set delay 100 \)
 *          +swap +delete opensim_double_pendulum_muscle_1.gif
 *      $ gifsicle --crop-transparency --optimize=O3 --colors=32 --delay 5 <
 *          opensim_double_pendulum_muscle_1.gif >
 *          opensim_double_pendulum_muscle.gif
 */

#define TESTING

#include <OpenSim/OpenSim.h>
using namespace SimTK;
using namespace OpenSim; using OpenSim::Body;
int main() {
    Model model;
#ifndef TESTING
    model.setUseVisualizer(true);
#endif
    // Two links, with mass of 1 kg, center of mass at the
    // origin of the body's frame, and moments/products of inertia of zero.
    OpenSim::Body* link1 = new OpenSim::Body("humerus", 1, Vec3(0), Inertia(0));
    OpenSim::Body* link2 = new OpenSim::Body("radius", 1, Vec3(0), Inertia(0));

    // Joints that connect the bodies together.
    PinJoint* joint1 = new PinJoint("shoulder",
            // Parent body, location in parent, orientation in parent.
            model.getGroundBody(), Vec3(0), Vec3(0),
            // Child body, location in child, orientation in child.
            *link1, Vec3(0, 1, 0), Vec3(0));
    PinJoint* joint2 = new PinJoint("elbow",
            *link1, Vec3(0), Vec3(0), *link2, Vec3(0, 1, 0), Vec3(0));

    // Add an actuator that crosses the elbow, and a joint stop.
    Millard2012EquilibriumMuscle* muscle = new
        Millard2012EquilibriumMuscle("biceps", 200, 0.6, 0.55, 0);
    muscle->addNewPathPoint("point1", *link1, Vec3(0, 0.8, 0));
    muscle->addNewPathPoint("point2", *link2, Vec3(0, 0.7, 0));

    // A controller that specifies the excitation of the biceps muscle.
    PrescribedController* brain = new PrescribedController();
    brain->addActuator(*muscle);
    // Muscle excitation is 0.3 for the first 0.5 seconds, and 1.0 thereafter.
    brain->prescribeControlForActuator("biceps",
            new StepFunction(0.5, 3, 0.3, 1));

    // Add bodies and joints to the model.
    model.addBody(link1); model.addBody(link2);
    model.addJoint(joint1); model.addJoint(joint2);
    model.addForce(muscle);
    model.addController(brain);

    // Configure the model.
    State& state = model.initSystem();
    // Fix shoulder joint, flex elbow joint.
    model.updCoordinateSet()[0].setLocked(state, true);
    model.updCoordinateSet()[1].setValue(state, 0.5 * Pi);
    model.equilibrateMuscles(state);

    // Add display geometry.
#ifndef TESTING
    model.updMatterSubsystem().setShowDefaultGeometry(true);
    Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundColor(Vec3(1, 1, 1));
    // Ellipsoids: 0.5 m radius along y axis, centered 0.5 m up along y axis.
    DecorativeEllipsoid geom(Vec3(0.1, 0.5, 0.1)); Vec3 center(0, 0.5, 0);
    viz.addDecoration(link1->getIndex(), Transform(center), geom);
    viz.addDecoration(link2->getIndex(), Transform(center), geom);
#endif

    // Simulate.
    RungeKuttaMersonIntegrator integrator(model.getSystem());
    Manager manager(model, integrator);
    manager.setInitialTime(0); manager.setFinalTime(10.0);
#ifndef TESTING
    getchar();
#endif
    manager.integrate(state);
};
