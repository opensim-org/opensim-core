/* -------------------------------------------------------------------------- *
 *                          OpenSim:  testBody.cpp                            *
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

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>

using namespace OpenSim;

int main() {

    // Make sure that zero mass with nonzero inertia gives an error message,
    // and the inertia is changed to zero.
    // The check is done in Body::finalizeFromProperties(), which gets invoked
    // from Model::initSystem().
    // It should also be invoked by copy construction, but this didn't work for
    // me. -chrisdembia
    Model model;
    Body * b1 = new Body();
    b1->setName("body1");
    b1->set_mass(0);
    b1->set_inertia(SimTK::Vec6(1, 0, 0, 0, 0, 0));
    SimTK::Vec3 v(0);
    WeldJoint* joint = new WeldJoint("joint", model.getGroundBody(), v, v,
            *b1, v, v);
    model.addBody(b1);
    model.addJoint(joint);
    SimTK::State& s = model.initSystem();
    SimTK_TEST_EQ(model.updBodySet().get("body1").get_inertia(),
            SimTK::Vec6(0));
    return 0;
}

