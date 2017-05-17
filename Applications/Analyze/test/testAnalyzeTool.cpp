/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testAnalyzeTool.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Ajay Seth                                          *
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

// Test different default activations are respected when activation
// states are not provided. Note if default_activation is 1.0 it 
// reproduces  the "skyscraper" bug issue #83 in OpenSim32 repo
void testTugOfWar(const string& dataFileName, const double& defaultAct);


int main()
{
    SimTK::Array_<std::string> failures;

    try { testTutorialOne(); }
    catch (const std::exception& e) {
        cout << e.what() << endl; failures.push_back("testTutorialOne");
    }
    // produce passive force-length curve
    try { testTugOfWar("Tug_of_War_ConstantVelocity.sto", 0.01); }
    catch (const std::exception& e) {
        cout << e.what() << endl; 
        failures.push_back("testTugOfWar CoordinatesOnly: default_act = 0.01");
    }
    //produced passive + active but before skyscraper bug appears (at 0.9)
    try { testTugOfWar("Tug_of_War_ConstantVelocity.sto", 0.8); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testTugOfWar CoordinatesOnly: default_act = 0.8");
    }
    // now supply activation states which should be used instead of the default
    try { testTugOfWar("Tug_of_War_ConstantVelocity_RampActivation.sto", 0.0); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testTugOfWar with activation state provided");
    }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
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

void testTugOfWar(const string& dataFileName, const double& defaultAct) {
    AnalyzeTool analyze("Tug_of_War_Setup_Analyze.xml");
    analyze.setCoordinatesFileName("");
    analyze.setStatesFileName("");

    // Access the model and muscle being analyzed
    Model& model = analyze.getModel();
    Millard2012EquilibriumMuscle& muscle =
        static_cast<Millard2012EquilibriumMuscle&>(model.updMuscles()[0]);
    bool isCoordinatesOnly = true;
    // Load in the States used to recompute the results of the Analysis 
    Storage dataStore(dataFileName);
    if (dataStore.getColumnLabels().findIndex(muscle.getName() + ".activation") > 0) {
        isCoordinatesOnly = false;
        analyze.setStatesFileName(dataFileName);
        // ramp input starts at 0.0
        muscle.set_minimum_activation(0.0);
    }
    else {
        analyze.setCoordinatesFileName(dataFileName);
    }

    // Test that the default activation is taken into consideration by
    // the Analysis and reflected in the AnalyzeTool solution
    muscle.set_default_activation(defaultAct);
    analyze.run();

    // Load the AnalyzTool results for the muscle's force through time
    TimeSeriesTable_<double> results =
        STOFileAdapter_<double>::
        read("Analyze_Tug_of_War/Tug_of_War_Millard_Iso_ForceReporter_forces.sto");
    assert(results.getNumColumns() == 1);
    SimTK::Vector forces = results.getDependentColumnAtIndex(0);

    // Load input data as StatesTrajectory used to perform the Analysis
    auto statesTraj = StatesTrajectory::createFromStatesStorage(
        model, dataStore, true, false);
    size_t nstates = statesTraj.getSize();

    // muscle active, passive, total muscle and tendon force quantities
    double af, pf, mf, tf;
    af= pf = mf = tf = SimTK::NaN;

    // Tolerance for muscle equilibrium solution 
    const double equilTol = muscle.getMaxIsometricForce()*SimTK::SqrtEps;

    SimTK::State s = model.getWorkingState();
    // Independently compute the active fiber force at every state
    for (int i = 0; i < nstates; ++i) {
        s = statesTraj[i];
        // When the muscle states are not supplied in the input dataStore
        // (isCoordinatesOnly == true), then set it to its default value.
        if (isCoordinatesOnly) {
            muscle.setActivation(s, muscle.get_default_activation());
        }
        // technically, fiber lengths could be supplied, but this test case
        // (a typical use case) does not and therefore set to its default.
        muscle.setFiberLength(s, muscle.get_default_fiber_length());
        muscle.computeEquilibrium(s);
        model.realizeDynamics(s);

        if (isCoordinatesOnly) {
            // check that activation is not reset to zero or other value
            SimTK_ASSERT_ALWAYS(muscle.getActivation(s) == defaultAct,
                "Test failed to correctly use the default activation value.");
        }
        else {
            // check that activation used was that supplied by the dataStore
            SimTK_ASSERT_ALWAYS(muscle.getActivation(s) == s.getTime(),
                "Test failed to correctly use the supplied activation values.");
        }
 
        // get active and passive forces given the default activation
        af = muscle.getActiveFiberForceAlongTendon(s);
        pf = muscle.getPassiveFiberForceAlongTendon(s);
        // now the total muscle force is the active + passive
        mf = af + pf;
        tf = muscle.getTendonForce(s);

        // equilibrium demands tendon and muscle fiber are equivalent
        ASSERT_EQUAL<double>(tf, mf, equilTol);
        // Verify that the current computed and AnalyzeTool reported force are
        // equivalent for the provided motion file
        cout << s.getTime() << " :: muscle-fiber-force: " << mf <<
            " Analyze reported force: " << forces[i] << endl;
        ASSERT_EQUAL<double>(mf, forces[i], equilTol, __FILE__, __LINE__,
            "Total fiber force failed to match reported muscle force.");
    }
}
