/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testRealizeDynamics.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Author(s): Thomas Lau                                                      *
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
#include <stdint.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Control/ControlSetController.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

//==============================================================================
// testRealizeDynamics tests that the results produced in every stage are 
// repeatable - this test is designed primarily to make sure that there are no 
// memory races during realizeDynamics that can cause time-dependent results 
// with regards to thread timing.
//==============================================================================
void testRealizeDynamics(const string& modelFile);

static const int MAX_N_TRIES = 100;

int main()
{
    try {
        testRealizeDynamics("arm26.osim");
    }
    catch (const Exception& e) {
        cout << "testRealizeDynamics failed: ";
        e.print(cout);
        return 1;
    }
    catch (const std::exception& e) {
        cout << "testRealizeDynamics failed: " << e.what() << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

//==============================================================================
// Test Cases
//==============================================================================
void testRealizeDynamics(const string& modelFile)
{
    using namespace SimTK;
    using namespace std;

    //==========================================================================
    // Setup OpenSim model
    Model model(modelFile);

    ControlSetController* controller = new ControlSetController();
    controller->setControlSetFileName("arm26_StaticOptimization_controls.xml");
    model.addController(controller);

    //==========================================================================
    // Test Accuracy of Dynamics
    
    //Realize to Stage::Dynamics once to obtain a Dyanmics "answer"
    State& state = model.initSystem();
    model.getSystem().realize(state, Stage::Dynamics);
    const MultibodySystem& mbs = model.getMultibodySystem();
    Vector_<SpatialVec>&   rigidBodyForcesAnswer = mbs.updRigidBodyForces(state,
                                                               Stage::Dynamics);
    string rigidBodyForcesAnswerString = rigidBodyForcesAnswer.toString();
    Vector_<Vec3>&         particleForcesAnswer = mbs.updParticleForces(state,
                                                               Stage::Dynamics);
    string particleForcesAnswerString = particleForcesAnswer.toString();
    Vector&                mobilityForcesAnswer = mbs.updMobilityForces(state,
                                                               Stage::Dynamics);
    string mobilityForcesAnswerString = mobilityForcesAnswer.toString();

    //Constantly realize to Stage::Dynamics to make sure that our answers are
    //consistent
    for(int x = 0; x < 100; x++)
    {
      State& state = model.initSystem();

      model.getSystem().realize(state, Stage::Dynamics);
      const MultibodySystem& mbs = model.getMultibodySystem();
      Vector_<SpatialVec>   tempRigidBodyForces = mbs.updRigidBodyForces(state,
                                                               Stage::Dynamics);
      Vector_<Vec3>&    tempParticleForcesAnswer = mbs.updParticleForces(state,
                                                               Stage::Dynamics);
      Vector&            tempMobilityForcesAnswer = mbs.updMobilityForces(state,
                                                               Stage::Dynamics);
                                                               
      SimTK_TEST((int)(rigidBodyForcesAnswerString == tempRigidBodyForces.toString()) == 1);
      SimTK_TEST((int)(particleForcesAnswerString == tempParticleForcesAnswer.toString()) == 1);
      SimTK_TEST((int)(particleForcesAnswerString == tempParticleForcesAnswer.toString()) == 1);

    }
}
