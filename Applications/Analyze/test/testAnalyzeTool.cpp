/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testAnalyzeTutorialOne.cpp                    *
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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/StatesTrajectory.h>
#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void testTutorialOne();
SimTK::Vector testTugOfWar_Passive();
void testTugOfWar_DefaultActivation();
void testTugOfWar_ActivationOverride();


int main()
{
    try {
        //testTutorialOne();
        SimTK::Vector passiveForces = testTugOfWar_Passive();
        testTugOfWar_DefaultActivation();
        testTugOfWar_ActivationOverride();
    }
    catch (const exception& e) {
        cout << "testTutorialOne Failed: " << e.what() << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

void testTutorialOne() {
    AnalyzeTool analyze1("PlotterTool.xml");
    analyze1.getModel().print("testAnalyzeTutorialOne.osim");
    analyze1.run();
    /* Once this runs to completion we'll make the test more meaningful by comparing output
    * to a validated standard. Let's make sure we don't crash during run first! -Ayman 5/29/12 */
    Storage resultFiberLength("testPlotterTool/BothLegs__FiberLength.sto");
    Storage standardFiberLength("std_BothLegs_fiberLength.sto");
    CHECK_STORAGE_AGAINST_STANDARD(resultFiberLength, standardFiberLength,
        std::vector<double>(100, 0.0001), __FILE__, __LINE__,
        "testAnalyzeTutorialOne failed");
    // const Model& mdl = analyze1.getModel();
    //mdl.updMultibodySystem()
    analyze1.setStatesFileName("plotterGeneratedStatesHip45.sto");
    //analyze1.setModel(mdl);
    analyze1.setName("BothLegsHip45");
    analyze1.run();
    Storage resultFiberLengthHip45("testPlotterTool/BothLegsHip45__FiberLength.sto");
    Storage standardFiberLength45("std_BothLegsHip45__FiberLength.sto");
    CHECK_STORAGE_AGAINST_STANDARD(resultFiberLengthHip45,
        standardFiberLength45, std::vector<double>(100, 0.0001),
        __FILE__, __LINE__, "testAnalyzeTutorialOne at Hip45 failed");
    cout << "testAnalyzeTutorialOne passed" << endl;
}

SimTK::Vector testTugOfWar_Passive() {
    AnalyzeTool analyze("Tug_of_War_Setup_Analyze.xml");
    Model& model = analyze.getModel();
    Millard2012EquilibriumMuscle& muscle =
        static_cast<Millard2012EquilibriumMuscle&>(model.updMuscles()[0]);
    //muscle.setFiberDamping(0.001);
    //set default activation to zero
    muscle.set_default_activation(0.0);
    // set min achievable activation to 0 as well
    muscle.set_minimum_activation(0.0);
    analyze.run();

    // Load the AnalyzTool results for the muscle's force through time
    TimeSeriesTable_<double> results =
        STOFileAdapter_<double>::
        read("Analyze_Tug_of_War/Tug_of_War_Millard_Iso_ForceReporter_forces.sto");
    assert(results.getNumColumns() == 1);
    SimTK::Vector forces = results.getDependentColumnAtIndex(0);

    // Load in the States used to perform the Analysis
    Storage statesSto("Tug_of_War_ConstantVelocity.sto");
    auto statesTraj = StatesTrajectory::createFromStatesStorage(
        model, statesSto, true, false);
    size_t nstates = statesTraj.getSize();

    assert(forces.size() == nstates);

    SimTK::Vector pfs(nstates, SimTK::NaN);

    SimTK::State s = model.getWorkingState();
    double tf = SimTK::NaN;
    // Independently compute the passive fiber force at every state
    for (int i = 0; i < nstates; ++i) {
        s = statesTraj[i];
        muscle.setActivation(s, 0.0);
        muscle.setFiberLength(s, muscle.get_default_fiber_length());
        muscle.computeEquilibrium(s);
        model.realizeDynamics(s);
        assert(muscle.getActivation(s) == 0.0);
        // At 0 activation, the fiber force must be given by its passive properties
        pfs[i] = muscle.getPassiveFiberForceAlongTendon(s);
        tf = muscle.getTendonForce(s);

        // equilibrium demands tendon and fiber are equivalent
        ASSERT_EQUAL<double>(tf, pfs[i], SimTK::SqrtEps);

        // Verify that the current computed and AnalyzeTool reported force are
        // equivalent
        //cout << "Passive-fiber-force: " << pfs[i] <<
        //    " Analyze muscle force: " << forces[i] << endl;
        ASSERT_EQUAL<double>(pfs[i], forces[i], SimTK::SqrtEps, __FILE__, __LINE__,
            "Passive fiber force failed to match reported muscle force.");
    }

    return pfs;
}

void testTugOfWar_DefaultActivation() {
    AnalyzeTool analyze("Tug_of_War_Setup_Analyze.xml");
    Model& model = analyze.getModel();
    Millard2012EquilibriumMuscle& muscle =
        static_cast<Millard2012EquilibriumMuscle&>(model.updMuscles()[0]);
    //muscle.setFiberDamping(0.0001);
    // Test that the default activation is taken into consideration by
    // the Analysis and that above was by chance due to 0 activation
    //set default activation to zero
    muscle.set_default_activation(0.8);
    analyze.run();

    // Load the AnalyzTool results for the muscle's force through time
    TimeSeriesTable_<double> results =
        STOFileAdapter_<double>::
        read("Analyze_Tug_of_War/Tug_of_War_Millard_Iso_ForceReporter_forces.sto");
    assert(results.getNumColumns() == 1);
    SimTK::Vector forces = results.getDependentColumnAtIndex(0);

    // Load in the States used to perform the Analysis
    Storage statesSto("Tug_of_War_ConstantVelocity.sto");
    auto statesTraj = StatesTrajectory::createFromStatesStorage(
        model, statesSto, true, false);
    size_t nstates = statesTraj.getSize();

    // muscle active, damping, total muscle and tendon force quantities
    double af, pf, mf, tf = SimTK::NaN;

    const double equilTol = muscle.getMaxIsometricForce()*SimTK::SqrtEps;

    SimTK::State s = model.getWorkingState();
    // Independently compute the active fiber force at every state
    for (int i = 0; i < nstates; ++i) {
        s = statesTraj[i];
        muscle.setActivation(s, muscle.get_default_activation());
        muscle.setFiberLength(s, muscle.get_default_fiber_length());
        muscle.computeEquilibrium(s);
        model.realizeDynamics(s);
        // make sure the activation is not be reset to zero
        assert(muscle.getActivation(s) == muscle.get_default_activation());
        // At default activation the active force is non-zero
        af = muscle.getActiveFiberForceAlongTendon(s);
        pf = muscle.getPassiveFiberForceAlongTendon(s);
        // now the total muscle force is the active + passive
        mf = af + pf; //  + df;
        tf = muscle.getTendonForce(s);

        // equilibrium demands tendon and muscle fiber are equivalent
        ASSERT_EQUAL<double>(tf, mf, equilTol);
        // Verify that the current computed and AnalyzeTool reported force are
        // equivalent
        cout << "Muscle-fiber-force: " << mf <<
            " Analyze muscle force: " << forces[i] << endl;
        ASSERT_EQUAL<double>(mf, forces[i], equilTol, __FILE__, __LINE__,
            "Total fiber force failed to match reported muscle force.");
    }
}

void testTugOfWar_ActivationOverride() {
    AnalyzeTool analyze("Tug_of_War_Setup_Analyze.xml");
    analyze.setCoordinatesFileName("");

    // Load in the States used to perform the Analysis
    Storage statesSto("Tug_of_War_ConstantVelocity_RampActivation.sto");
    analyze.setStatesFileName("Tug_of_War_ConstantVelocity_RampActivation.sto");

    Model& model = analyze.getModel();
    Millard2012EquilibriumMuscle& muscle =
        static_cast<Millard2012EquilibriumMuscle&>(model.updMuscles()[0]);
    analyze.run();

    // Load the AnalyzTool results for the muscle's force through time
    TimeSeriesTable_<double> results =
        STOFileAdapter_<double>::
        read("Analyze_Tug_of_War/Tug_of_War_Millard_Iso_ForceReporter_forces.sto");
    assert(results.getNumColumns() == 1);
    SimTK::Vector forces = results.getDependentColumnAtIndex(0);

    auto statesTraj = StatesTrajectory::createFromStatesStorage(
        model, statesSto, true, false);
    size_t nstates = statesTraj.getSize();

    assert(forces.size() == nstates);

    // muscle active, damping, total muscle and tendon force quantities
    double af, pf, mf, tf = SimTK::NaN;

    const double equilTol = muscle.getMaxIsometricForce()*SimTK::SqrtEps;
    SimTK::State s = model.getWorkingState();
    // At the first State, independently compute the muscle force
    for (int i = 0; i < nstates; ++i) {
        s = statesTraj[i];
        muscle.setFiberLength(s, muscle.get_default_fiber_length());
        muscle.computeEquilibrium(s);
        model.realizeDynamics(s);
        // activation is not reset to zero and must reflect the read-in state
        assert(muscle.getActivation(s) > 0.0);
        // At default activation the active force is non-zero
        af = muscle.getActiveFiberForceAlongTendon(s);
        pf = muscle.getPassiveFiberForceAlongTendon(s);
        tf = muscle.getTendonForce(s);
        // now the total muscle force is the active + passive
        mf = af + pf;

        // equilibrium demands tendon and muscle fiber are equivalent
        ASSERT_EQUAL<double>(tf, mf, equilTol);
        // Verify that the current computed and AnalyzeTool reported force are
        // equivalent
        cout << "Muscle-fiber-force: " << mf <<
            " Analyze muscle force: " << forces[i] << endl;
        ASSERT_EQUAL<double>(mf, forces[i], equilTol, __FILE__, __LINE__,
            "Total fiber force failed to match reported muscle force.");
    }
}


