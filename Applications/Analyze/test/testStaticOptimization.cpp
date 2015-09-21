/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testStaticOptimization.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Matthew Millard, Ajay Seth                         *
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
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Analyses/StaticOptimization.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

/** @param muscleModelClassName selects from:
        Thelen2003Muscle_Deprecated
        Thelen2003Muscle
        Millard2012EquilibriumMuscle
        Millard2012AccelerationMuscle
*/
void testArm26(const string& muscleModelClassName, double atol, double ftol);

void testLapackErrorDLASD4();

void testModelWithPassiveForces();

int main()
{
    Array<string> muscleModelNames;
    muscleModelNames.append("Thelen2003Muscle_Deprecated");
    muscleModelNames.append("Thelen2003Muscle");
    muscleModelNames.append("Millard2012EquilibriumMuscle");
    muscleModelNames.append("Millard2012AccelerationMuscle");
    
    // Tolerances for the differences between the current models
    // and the 'standard' solution, which was closest to using 
    // Thelen2003Muscle_Deprecated muscle formulation.
    double actTols[4] = {0.005, 0.025, 0.04, 0.04};
    double forceTols[4] = {0.5, 4, 5, 6};
    
    SimTK::Array_<std::string> failures;
    
    for(int i=0; i< muscleModelNames.getSize(); ++i){
        try { // regression test for the Thelen deprecate muscle
           // otherwise verify that SO runs with the new models
            testArm26(muscleModelNames[i], actTols[i], forceTols[i]);
        }
        catch (const std::exception& e) {
            cout << e.what() <<endl; 
            failures.push_back("testArm26_"+muscleModelNames[i]);
        }
    }
    
    try {
        testModelWithPassiveForces();
    }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testModelWithPassiveForces");
    }
    
    try {
        testLapackErrorDLASD4();
    }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testLapackErrorDLASD4");
    }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done" << endl;
    return 0;
}

void testArm26(const string& muscleModelClassName, double actTol, double forceTol)
{
    Object::renameType( "Thelen2003Muscle", muscleModelClassName);

    cout << "==============================================" << endl;
    cout << "       "<< muscleModelClassName << endl;
    cout << "==============================================" << endl;

    string std_force = "std_";
    string std_activation = "std_";
    string std_bounds_force = "std_";
    string std_bounds_activation = "std_";

    std_force.append(muscleModelClassName);
    std_activation.append(muscleModelClassName);
    std_bounds_force.append(muscleModelClassName);
    std_bounds_activation.append(muscleModelClassName);

    std_force.append("_arm26_StaticOptimization_force.sto");
    std_activation.append("_arm26_StaticOptimization_activation.sto");
    std_bounds_force.append("_arm26_bounds_StaticOptimization_force.sto");
    std_bounds_activation.append("_arm26_bounds_StaticOptimization_activation.sto");

    string resultsDir = "Results_"+muscleModelClassName;
    const string& muscName = muscleModelClassName;

    AnalyzeTool analyze1("arm26_Setup_StaticOptimization.xml");
    analyze1.setResultsDir(resultsDir);
    analyze1.run();

    Storage activations1(resultsDir+"/arm26_StaticOptimization_activation.sto");
    Storage stdActivations1(std_activation);
    // Uncomment to use muscle model specific standard
    //Storage stdActivations1("std_arm26_"+muscName+"_SO_activation.sto");

    Storage forces1(resultsDir+"/arm26_StaticOptimization_force.sto");
    Storage stdForces1(std_force);
    // Uncomment to use muscle model specific standard
    //Storage stdForces1("std_arm26_"+muscName+"_SO_force.sto");

    CHECK_STORAGE_AGAINST_STANDARD(activations1, stdActivations1, 
                                    Array<double>(actTol, 6),
                                    __FILE__, __LINE__, 
                                    "Arm26 activations "+muscName+" failed");

    CHECK_STORAGE_AGAINST_STANDARD(forces1, stdForces1, 
                                    Array<double>(forceTol, 6),
                                    __FILE__, __LINE__, 
                                    "Arm26 forces "+muscName+" failed.");
    cout << resultsDir <<": test Arm26 passed." << endl;
  
    
    cout << "=============================================================\n" << endl;

    AnalyzeTool analyze2("arm26_bounds_Setup_StaticOptimization.xml");
    analyze2.setResultsDir(resultsDir);
    analyze2.run();

    Storage activations2(
        resultsDir+"/arm26_bounds_StaticOptimization_activation.sto");
    Storage stdActivations2(std_bounds_activation); 
    // Uncomment to use muscle model specific standard
    //Storage stdActivations2("std_arm26_bounds_"+muscName+"_SO_activation.sto");

    Storage forces2(resultsDir+"/arm26_bounds_StaticOptimization_force.sto");
    Storage stdForces2(std_bounds_force);
    // Uncomment to use muscle model specific standard
    //Storage stdForces2("std_arm26_bounds_"+muscName+"_SO_force.sto");

    CHECK_STORAGE_AGAINST_STANDARD(activations2, stdActivations2,
        Array<double>(actTol, 6), 
        __FILE__, __LINE__, 
        "Arm26 activation "+muscName+" with bounds failed.");

    CHECK_STORAGE_AGAINST_STANDARD(forces2, stdForces2, 
        Array<double>(forceTol, 6),
        __FILE__,  __LINE__, 
        "Arm26 forces "+muscName+" with bounds failed.");
 
    cout << resultsDir << ": testArm26 with bounds passed" << endl;
    cout << "=============================================================\n" << endl;
}

void testModelWithPassiveForces() {
    AnalyzeTool analyze("staticoptimization_spring_Setup.xml");
    analyze.run();
    std::string resultsDir("ResultsSO_spring");
    Storage activations(resultsDir + "/walk_subject01_ankle_spring_StaticOptimization_activation.sto");
    Storage stdActivations("std_walk_subject01_ankle_spring_StaticOptimization_activation.sto");

    Storage forces(resultsDir + "/walk_subject01_ankle_spring_StaticOptimization_force.sto");
    Storage stdForces("std_walk_subject01_ankle_spring_StaticOptimization_force.sto");

    CHECK_STORAGE_AGAINST_STANDARD(activations, stdActivations,
        Array<double>(0.025, 28),
        __FILE__, __LINE__,
        "ModelWithPassiveForces activations failed");

    CHECK_STORAGE_AGAINST_STANDARD(forces, stdForces,
        Array<double>(2.5, 48),
        __FILE__, __LINE__,
        "ModelWithPassiveForces forces failed.");
    cout << resultsDir << ": test ModelWithPassiveForces passed." << endl;

}

void testLapackErrorDLASD4() {
    // With OpenSim 3.2 64bit, the 64 bit lapack library (in Simbody 3.3.1) 
    // crashes with an error[1] if there are not enough actuators (or under 
    // other similar circumstances). With Simbody 3.5.2, the 64 bit Windows lapack
    // libraries are updated and do not have this bug.
    // This test ensures that the bug is gone. We only expect this test to fail
    // on Windows 64 bit build using Simbody 3.3.1 or earlier.
    // What this function should do is print SimTK exceptions about not being 
    // able to satisfy constraints, but nothing should crash and 
    // no exceptions should reach this function.
    // 
    // The error is caused by using unfiltered inverse kinematics as input.
    //
    // [1] The error was:
    //     "** On entry to DLASD4 parameter number -1 had an illegal value"
    AnalyzeTool analyze("subject01_Setup_StaticOptimization.xml");
    analyze.setResultsDir("Results_subject01_StaticOptimization_LapackError");
    analyze.run();
}
