/* -------------------------------------------------------------------------- *
 *                      OpenSim:  testJointReactions.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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

// INCLUDE
#include <OpenSim/OpenSim.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void testJointReactionDuringIterativeForwardSimulation();

int main()
{
    try {
        AnalyzeTool analyze("SinglePin_Setup_JointReaction.xml");
        analyze.run();
        Storage result1("SinglePin_JointReaction_ReactionLoads.sto"), 
            standard1("std_SinglePin_JointReaction_ReactionLoads.sto");
        CHECK_STORAGE_AGAINST_STANDARD(result1, standard1, 
            std::vector<double>(standard1.getSmallestNumberOfStates(), 1e-5),
            __FILE__, __LINE__, "SinglePin failed");
        cout << "SinglePin passed" << endl;

        AnalyzeTool analyze2("DoublePendulum3D_Setup_JointReaction.xml");
        analyze2.run();
        Storage result2("DoublePendulum3D_JointReaction_ReactionLoads.sto"), 
            standard2("std_DoublePendulum3D_JointReaction_ReactionLoads.sto");
        CHECK_STORAGE_AGAINST_STANDARD(result2, standard2, 
            std::vector<double>(standard2.getSmallestNumberOfStates(), 1e-5), 
            __FILE__, __LINE__, "DoublePendulum3D failed");
        cout << "DoublePendulum3D passed" << endl;

        // check if joint reaction analysis behaves correctly during
        // iterative forward simulation
        testJointReactionDuringIterativeForwardSimulation();
    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

void testJointReactionDuringIterativeForwardSimulation() {
    Model model;

    // toy model
    auto ground = model.updGround();
    double m = 1, r = 0.01, l = 1;
    auto body = new OpenSim::Body("body1", m, SimTK::Vec3(0), 
        m * SimTK::Inertia::cylinderAlongY(r, l / 2));
    SimTK::Vec3 body_start(0, -l / 2, 0);
    SimTK::Vec3 body_end(0, l / 2, 0);
    auto joint = new OpenSim::PinJoint("joint1",
        ground, SimTK::Vec3(0), SimTK::Vec3(0),
        *body, body_start, SimTK::Vec3(0));
    joint->upd_coordinates(0).
        setDefaultValue(SimTK::convertDegreesToRadians(45));
    auto geom = new OpenSim::Cylinder(r, l / 2);
    geom->setName("cylinder");
    body->attachGeometry(geom);
    model.addBody(body);
    model.addJoint(joint);

    // analysis
    JointReaction* reaction = new JointReaction(&model);
    model.addAnalysis(reaction);

    // build model
    //model.setUseVisualizer(true);
    model.finalizeFromProperties();
    model.buildSystem();
    auto s = model.initializeState();

    double dt = 0.01;
    double t_start = 0;
    double t_end = 0.5;
    SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
    Manager manager(model, integrator);
    manager.setInitialTime(t_start);

    // iterative numerical integration  
    for (unsigned int i = 1; i*dt <= t_end; ++i) {
        double t0 = s.getTime();
        double tf = i * dt;

        // do something ...

        manager.setFinalTime(tf);
        manager.integrate(s);
        manager.setInitialTime(tf);
    }
    // if storage is reseted then it will not start from start time and the bug
    // is exposed
    ASSERT(reaction->getReactionLoads().getFirstTime() == t_start);
}
