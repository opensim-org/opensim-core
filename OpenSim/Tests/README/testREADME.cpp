/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testREADME.cpp                                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Chris Dembia                                                    *
 * Contributor(s): Thomas Uchida, James Dunne                                 *
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

#include <OpenSim/OpenSim.h>
using namespace SimTK;
using namespace OpenSim;
int main() {
    Model model;
#ifdef VISUALIZE
    model.setUseVisualizer(true);
#endif

    // Create two links, each with a mass of 1 kg, center of mass at the body's
    // origin, and moments and products of inertia of zero.
    OpenSim::Body* femur = new OpenSim::Body("femur", 1, Vec3(0), Inertia(0));
    OpenSim::Body* tibia = new OpenSim::Body("tibia", 1, Vec3(0), Inertia(0));

    // Connect the bodies with pin joints. Assume each body is 1 m long.
    PinJoint* hip = new PinJoint("hip",
            // Parent body, location in parent, orientation in parent.
            model.getGround(), Vec3(0), Vec3(0),
            // Child body, location in child, orientation in child.
            *femur, Vec3(0, 1, 0), Vec3(0));
    PinJoint* knee = new PinJoint("knee",
            *femur, Vec3(0), Vec3(0), *tibia, Vec3(0, 1, 0), Vec3(0));

    // Add a muscle that flexes the knee.
    Millard2012EquilibriumMuscle* muscle = new
        Millard2012EquilibriumMuscle("biceps", 200, 0.6, 0.55, 0);
    muscle->addNewPathPoint("origin",    *femur, Vec3(0, 0.8, 0));
    muscle->addNewPathPoint("insertion", *tibia, Vec3(0, 0.7, 0));

    // Add a controller that specifies the excitation of the muscle.
    PrescribedController* brain = new PrescribedController();
    brain->addActuator(*muscle);
    // Muscle excitation is 0.3 for the first 0.5 seconds, then increases to 1.
    brain->prescribeControlForActuator("biceps",
            new StepFunction(0.5, 3, 0.3, 1));

    // Add components to the model.
    model.addBody(femur); model.addBody(tibia);
    model.addJoint(hip);  model.addJoint(knee);
    model.addForce(muscle);
    model.addController(brain);

    // Add a console reporter to print the muscle fiber force and knee angle.
    ConsoleReporter* reporter = new ConsoleReporter();
    reporter->set_report_time_interval(1.0);
    reporter->updInput("inputs").connect(muscle->getOutput("fiber_force"));
    reporter->updInput("inputs").connect(
        knee->getCoordinateSet()[0].getOutput("value"));
    model.addComponent(reporter);

    // Configure the model.
    State& state = model.initSystem();
    // Fix the hip at its default angle and begin with the knee flexed.
    model.updCoordinateSet()[0].setLocked(state, true);
    model.updCoordinateSet()[1].setValue(state, 0.5 * Pi);
    model.equilibrateMuscles(state);

    // Add display geometry.
#ifdef VISUALIZE
    model.updMatterSubsystem().setShowDefaultGeometry(true);
    Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundColor(White);
    // Ellipsoids: 0.5 m radius along y-axis, centered 0.5 m up along y-axis.
    DecorativeEllipsoid geom(Vec3(0.1, 0.5, 0.1)); Vec3 center(0, 0.5, 0);
    viz.addDecoration(femur->getMobilizedBodyIndex(), Transform(center), geom);
    viz.addDecoration(tibia->getMobilizedBodyIndex(), Transform(center), geom);
#endif

    // Simulate.
    RungeKuttaMersonIntegrator integrator(model.getSystem());
    Manager manager(model, integrator);
    manager.setInitialTime(0); manager.setFinalTime(10.0);
#ifdef VISUALIZE // To give you the chance to click View -> Save Movie.
    std::cout << "Press enter/return to begin the simulation..." << std::endl;
    getchar();
#endif
    manager.integrate(state);
};
