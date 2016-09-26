/* -------------------------------------------------------------------------- *
 *                      OpenSim:  testNested<odel.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Ajay Seth,                                                      *
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

/*=============================================================================

Tests Include:
    1. Pendulum Model with nested Device derived from ModelComponent 
    1. Pendulum Model with a Device that is a Model

//=============================================================================*/
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

// Create Device as Concrete Container Component (like Model) of Components
class Device : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(Device, ModelComponent);
};


//==============================================================================
// Test Case Driver
//==============================================================================
template<class C>
void testPendulumModelWithNestedJoints()
{
    using namespace SimTK;
    Vec3 tolerance(SimTK::Eps);

    cout << "Running testPendulumModelWithNestedJoints<" << 
        typeid(C).name() << ">" << endl;

    // Load the pendulum model
    Model* pendulum = new Model("double_pendulum.osim");
    
    // Create a new empty device;
    C* device = new C();
    device->setName("device");

    // Build the device
    // Create bodies 
    auto* cuffA = new OpenSim::Body("cuffA", 1.0, Vec3(0), Inertia(0.5));
    auto* cuffB = new OpenSim::Body("cuffB", 1.0, Vec3(0), Inertia(0.5));

    // add Bodies to the device
    device->addComponent(cuffA);
    device->addComponent(cuffB);

    // Create WeldJoints to anchor cuff Bodies to the pendulum.
    auto* anchorA = new WeldJoint();
    anchorA->setName("anchorA");
    anchorA->updConnector("child_frame").connect(*cuffA);

    auto* anchorB = new WeldJoint();
    anchorB->setName("anchorB");
    anchorB->updConnector("child_frame").connect(*cuffB);

    // add anchors to the Device
    device->addComponent(anchorA);
    device->addComponent(anchorB);

    // add the device to the pendulum model
    pendulum->addModelComponent(device);

    // Connect the device to the pendulum
    auto bodies = pendulum->getComponentList<OpenSim::Body>();
    auto& body = bodies.begin();
    anchorA->updConnector("parent_frame").connect(*body);
    body++;
    anchorB->updConnector("parent_frame").connect(*body);

    State& s = pendulum->initSystem();
}

int main()
{
    SimTK::Array_<std::string> failures;

    try { testPendulumModelWithNestedJoints<Device>(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testPendulumModelWithNestedJoints<Device>");
    }
    try { testPendulumModelWithNestedJoints<Model>(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testPendulumModelWithNestedJoints<Model>");
    }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done. All cases passed." << endl;

    return 0;
}