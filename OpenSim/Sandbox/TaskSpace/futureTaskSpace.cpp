/* ------------------------------------------------------------------------- *
*             OpenSim:  testTasSpace.cpp                                     *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2017 Stanford University and the Authors                *
* Author(s): Dimitar Stanev                                                  *
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

/*
Demonstrates the usage of task space control for a desired task that is specified
by a desired acceleration. The underlying computations are performed by the
Simbody TaskSpace implementation, which was altered, so as to be used by the
OpenSim engine. The TaskSpace operators were substituted by object oriented class
calls, as a drawback of the current implementation. For this test the arm's end
effector is specified to have a constant acceleration in the y direction and
the gravity compensation is considered.
*/

#include <iostream>

#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Simulation/Manager/Manager.h>

#include "TaskBasedController.h"
#include "TaskSpace.h"

using namespace OpenSim;
using namespace SimTK;

void switchMuscles(Model& model, State& state, bool appliesForce);
void testTaskSpace();

//#define PAUSE

int main()
{

    try {

        testTaskSpace();
    }
    catch (const std::exception& ex)
    {
        std::cout << "Exception: " << ex.what() << std::endl;
#ifdef PAUSE
        system("pause");
#endif
        return 1;
    }
    catch (...)
    {
        std::cout << "Unrecognized exception " << std::endl;
#ifdef PAUSE
        system("pause");
#endif
        return 1;
    }

#ifdef PAUSE
    system("pause");
#endif

    return 0;
}

void switchMuscles(Model& model, State& state, bool appliesForce)
{
    for (int i = 0; i < model.updMuscles().getSize(); i++)
    {
        model.updMuscles().get(i).setAppliesForce(state, appliesForce);
    }
}

void testTaskSpace()
{
    Model model("futureTaskSpace.osim");

    std::string indexBodyName = "hand_r";
    Vec3 indexOffset(0.05, -0.14, 0.011);

    Kinematics* kinematics = new Kinematics(&model);
    model.addAnalysis(kinematics);

    TaskSpace* taskSpace = new TaskSpace();
    model.addModelComponent(taskSpace);

    OpenSim::TaskBasedController* forceController =
        new OpenSim::TaskBasedController(*taskSpace,
            indexBodyName, indexOffset);
    model.addForce(forceController);

    model.buildSystem();
    State& s = model.initializeState();
    switchMuscles(model, s, false);

    double dt = 0.01;
    double t_start = 0;
    double t_end = 0.4;

    //setup integrator
    SimTK::RungeKuttaMersonIntegrator integrator(
        model.getMultibodySystem());
    integrator.setAccuracy(1E-7);

    //manager
    OpenSim::Manager manager(model, integrator);
    s.setTime(t_start);

    for (unsigned int i = 1; i*dt < t_end; ++i)
    {
        std::cout << "Integrate " << i * dt << std::endl;

        manager.integrate(s, i*dt);

        forceController->setAcceleration(Vec3(0, 10, 0));
    }

    //store results
    kinematics->printResults("TaskSpace");
}