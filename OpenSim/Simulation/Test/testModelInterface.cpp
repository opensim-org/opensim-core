/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testModelInterface.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>

using namespace OpenSim;
using namespace std;


int main() {
    LoadOpenSimLibrary("osimActuators");

    try {
        // the specific model used here is not important
        Model model("arm26.osim");
        model.finalizeFromProperties();

        // model should be up-to-date with its properties
        ASSERT(model.isObjectUpToDateWithProperties());

        // make an edit to a Component contained in the model's Set
        model.updMuscles()[0].upd_min_control() = 0.02;
        ASSERT(!model.isObjectUpToDateWithProperties());

        model.finalizeFromProperties();
        ASSERT(model.isObjectUpToDateWithProperties());

        // make an edit to a Component contained in another Set
        model.updBodySet()[1].upd_mass() = 0.05;
        ASSERT(!model.isObjectUpToDateWithProperties());

        model.finalizeFromProperties();
        ASSERT(model.isObjectUpToDateWithProperties());

        // make an edit through model's ComponentList access
        for (auto& body : model.updComponentList<Body>()) {
            body.upd_mass_center() = SimTK::Vec3(0);
        }
        ASSERT(!model.isObjectUpToDateWithProperties());

        SimTK::State dummy_state;

        ASSERT_THROW(ComponentHasNoSystem, model.getSystem());
        ASSERT_THROW(ComponentHasNoSystem, model.realizeDynamics(dummy_state));
        ASSERT_THROW(ComponentHasNoSystem, 
                     model.computeStateVariableDerivatives(dummy_state));
        // should be able to create a Manager either
        ASSERT_THROW(ComponentHasNoSystem,
                     Manager manager(model));

        // get a valid state
        SimTK::State state = model.initSystem();
        Manager manager(model);
        state.setTime(0.0);

        // this should invalidate the System
        model.finalizeFromProperties();

        // verify that finalizeFromProperties() wipes out the underlying System
        ASSERT_THROW(ComponentHasNoSystem, model.getSystem());

        // should not be able to "trick" the manager into integrating a model
        // given a stale but compatible state
        ASSERT_THROW(ComponentHasNoSystem, manager.integrate(state, 1.));

        state = model.initSystem();
        //re-establish all the connections in the model
        model.finalizeConnections(model);

        // verify that finalizeConnections() does not wipe out the underlying 
        // System iff there are no changes to the dependencies
        auto& sys = model.getSystem();

        auto elbowInHumerus = new PhysicalOffsetFrame("elbow_in_humerus",
            model.getComponent<Body>("r_humerus"),
            SimTK::Transform(SimTK::Vec3(0, -0.33, 0.0)) );

        model.addComponent(elbowInHumerus);

        // establish the new offset frame as part of the model but not
        // used by any joints, constraints or forces
        state = model.initSystem();

        // now update the elbow and connect to the new frame
        Joint& elbow = model.updComponent<Joint>("r_elbow");
        elbow.connectSocket_parent_frame(*elbowInHumerus);

        // satisfy the new connections in the model
        model.finalizeConnections(model);

        // now changing the connections should invalidate the System
        ASSERT_THROW(ComponentHasNoSystem, model.getSystem());

        // verify the new connection was made
        ASSERT(model.getComponent<Joint>("r_elbow").getParentFrame().getName()
                == "elbow_in_humerus");
    }
    catch (const std::exception& ex) {
        std::cout << ex.what() << std::endl;
        return 1;
    }

    cout << "Done" << endl;

    return 0;
}




