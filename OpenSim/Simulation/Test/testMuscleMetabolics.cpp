/* -------------------------------------------------------------------------- *
 *                  OpenSim:  testMuscleMetabolics.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Thomas Uchida, Chris Dembia                                     *
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

#include <Simbody.h>

#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Simulation/Model/UchidaUmbergerMuscleMetabolics.h>

const bool DISPLAY_PROBE_OUTPUTS      = false;
const bool DISPLAY_ERROR_CALCULATIONS = false;
const bool OUTPUT_FILES               = false;
const int numPoints = 21;

using namespace OpenSim;
using namespace SimTK;
using namespace std;

// This header relies on the constant globals and macros above.
#include "MuscleMetabolicsHelper.h"

// TODO create a helper function that will convert the old probe to the new one.
// TODO also create a helper that will create probes for a given model (for all of its muscles).
// TODO use muscleEffortScaling factor properly!
// TODO should the connectee just be the name of the MuscleRep?
// TODO clip active fiber force to be non-negative?
// TODOdiscuss how to set internal input connections using muscle connector.
// TODOdiscuss allow musclerep to be standalone?

void testInterface() {
    
    const double sol_maxIsometricForce  = 3127;
    const double sol_optimalFiberLength = 0.055;
    const double sol_Arel               = 0.18;
    const double sol_Brel               = 2.16;
    const double sol_muscleMass         = 0.805;
    
    // Create model.
    Model model = createSlidingBlockWithMuscle("soleus", sol_maxIsometricForce,
                                               sol_optimalFiberLength, 0.80,
                                               sol_Arel, sol_Brel, 1.5);
    model.initSystem();
    
    // Create metabolics component.
    auto* met = new UchidaUmbergerMuscleMetabolics();
    met->setName("metabolics");
    
    const int sol_repIdx = met->append_muscle_reps(
            UchidaUmbergerMuscleMetabolics::MuscleRep());
    auto& sol_rep = met->upd_muscle_reps(sol_repIdx);
    sol_rep.setName("soleus_met");
    sol_rep.set_provided_muscle_mass(sol_muscleMass);
    
    model.addModelComponent(met);
    
    // Add reporter.
    auto* reporter = new ConsoleReporter();
    reporter->setName("rep");
    reporter->set_report_time_interval(0.1);
    model.addComponent(reporter);
    
    // Set connections.
    const auto& soleus = model.getMuscles().get("soleus");
    sol_rep.updConnector<Muscle>("muscle").connect(soleus);
    
    auto& input = reporter->updInput("inputs");
    input.connect(soleus.getOutput("activation"));
    input.connect(soleus.getOutput("fiber_velocity"));
    input.connect(met->getOutput("whole_body_rate"));
    input.connect(met->getOutput("basal_rate"));
    input.connect(met->getOutput("soleus_met/total_rate"));
    input.connect(met->getOutput("soleus_met/activation_maintenance_rate"));
    input.connect(met->getOutput("soleus_met/shortening_rate"));
    input.connect(met->getOutput("soleus_met/mechanical_work_rate"));
    
    // TODO for debugging.
    model.dumpSubcomponents(0);
    
    // Simulate.
    SimTK::State& state = model.initSystem();
    SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
    Manager manager(model, integrator);
    manager.setInitialTime(0);
    manager.setFinalTime(1.0);
    manager.integrate(state);
    
}

int main() {
    SimTK_START_TEST("testMuscleMetabolics");
        SimTK_SUBTEST(testInterface);
    SimTK_END_TEST();
}

// This method reveals the bug where it's necessary to call initSystem() twice
// to set the ground mobilizer index before the joints need it.
void TODODoubleInitSystemBug() {
    
    const double sol_maxIsometricForce  = 3127;
    const double sol_optimalFiberLength = 0.055;
    const double sol_Arel               = 0.18;
    const double sol_Brel               = 2.16;
    const double sol_muscleMass         = 0.805;
    
    // Create model.
    Model model = createSlidingBlockWithMuscle("soleus", sol_maxIsometricForce,
                                               sol_optimalFiberLength, 0.80,
                                               sol_Arel, sol_Brel, 1.5);
    model.initSystem();
    
    // Create metabolics component.
    auto* met = new UchidaUmbergerMuscleMetabolics();
    met->setName("metabolics");
    
    auto sol_rep = UchidaUmbergerMuscleMetabolics::MuscleRep();
    sol_rep.setName("soleus_met");
    sol_rep.updConnector<Muscle>("muscle").connect(model.getMuscles().get("soleus"));
    sol_rep.set_provided_muscle_mass(sol_muscleMass);
    met->append_muscle_reps(sol_rep);
    
    model.addModelComponent(met);
    
    // Add reporter.
    auto* reporter = new ConsoleReporter();
    reporter->setName("rep");
    model.addComponent(reporter);
    
    // Set connections.
    reporter->updInput("inputs").connect(met->getOutput("soleus_met/total_rate"));
    reporter->updInput("inputs").connect(met->getOutput("soleus_met/mechanical_work_rate"));
    reporter->updInput("inputs").connect(met->getOutput("basal_rate"));
    reporter->updInput("inputs").connect(met->getOutput("whole_body_rate"));
    
    // Simulate.
    SimTK::State& state = model.initSystem();
    SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
    Manager manager(model, integrator);
    manager.setInitialTime(0);
    manager.setFinalTime(1.0);
    manager.integrate(state);
    
}