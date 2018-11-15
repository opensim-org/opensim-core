/* -------------------------------------------------------------------------- *
 *                   OpenSim:  buildTestbedModel.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Chris Dembia, Shrinidhi K. Lakshmikanth, Ajay Seth,             *
 *            Thomas Uchida                                                   *
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

/* Helper methods to take care of some mundane tasks. You don't need to add
anything in this file, but you should know what each of these methods does. */

#include <OpenSim/OpenSim.h>

namespace OpenSim {

//------------------------------------------------------------------------------
// Build a testbed for testing the device before attaching it to the hopper. We
// will attach one end of the device to ground ("/testbed/ground") and the other
// end to a sprung load ("/testbed/load").
//------------------------------------------------------------------------------
Model buildTestbed(bool showVisualizer)
{
    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create a new OpenSim model.
    auto testbed = Model();
    testbed.setName("testbed");
    if (showVisualizer)
        testbed.setUseVisualizer(true);
    testbed.setGravity(Vec3(0));

    // Create a 2500 kg load and add geometry for visualization.
    auto load = new Body("load", 2500., Vec3(0), Inertia(1.));
    auto sphere = new Sphere(0.02);
    sphere->setFrame(*load);
    sphere->setOpacity(0.5);
    sphere->setColor(SimTK::Blue);
    load->attachGeometry(sphere);
    testbed.addBody(load);

    // Attach the load to ground with a FreeJoint and set the location of the
    // load to (1,0,0).
    auto gndToLoad = new FreeJoint("gndToLoad", testbed.getGround(), *load);
    gndToLoad->updCoordinate(FreeJoint::Coord::TranslationX).setDefaultValue(1.0);
    testbed.addJoint(gndToLoad);

    // Add a spring between the ground's origin and the load.
    auto spring = new PointToPointSpring(
        testbed.getGround(), Vec3(0),   //frame G and location in G of point 1
        *load, Vec3(0),                 //frame F and location in F of point 2
        5000., 1.);                     //stiffness and rest length
    testbed.addForce(spring);

    return testbed;
}

} // end of namespace OpenSim
