/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testStaticOptimization.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Analyses/StaticOptimization.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <opensim-core/tests/opensim_tests_config.h>

#include <catch2/catch_all.hpp>

#include <filesystem>

using namespace OpenSim;
using namespace std;

namespace {
    void testArm26(const string& muscleModelClassName, double actTol,
            double forceTol) {
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

        const auto arm26Dir = OpenSim::opensim_tests_resources_directory() / "models" / "Arm26";
        // Absolute results dir under the current (build) working directory: the tool
        // writes results relative to the Setup XML's directory, which now lives in
        // the read-only resources tree, so an absolute path keeps output in the CWD.
        const string resultsDir =
            (std::filesystem::current_path() / ("Results_"+muscleModelClassName)).string();
        const string& muscName = muscleModelClassName;

        AnalyzeTool analyze1((arm26Dir / "arm26_Setup_StaticOptimization.xml").string());
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
                                    std::vector<double>(6, actTol),
                                    __FILE__, __LINE__,
                                    "Arm26 activations "+muscName+" failed");

        CHECK_STORAGE_AGAINST_STANDARD(forces1, stdForces1,
                                    std::vector<double>(6, forceTol),
                                    __FILE__, __LINE__,
                                    "Arm26 forces "+muscName+" failed.");
        cout << resultsDir <<": test Arm26 passed." << endl;


        cout << "=============================================================\n" << endl;

        // Reproduce the arm26_bounds scenario programmatically instead of shipping a
        // separate model + setup file: the old arm26_bounds.osim was identical to
        // arm26.osim except for explicit muscle control bounds [0.01, 1.0], and the
        // bounds setup ran over the time window [0.40, 0.90].
        AnalyzeTool analyze2((arm26Dir / "arm26_Setup_StaticOptimization.xml").string());
        analyze2.setName("arm26_bounds");
        analyze2.setResultsDir(resultsDir);
        for (auto& muscle : analyze2.getModel().updComponentList<Muscle>()) {
            muscle.set_min_control(0.01);
            muscle.set_max_control(1.0);
        }
        analyze2.setInitialTime(0.40);
        analyze2.setFinalTime(0.90);
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
            std::vector<double>(6, actTol),
            __FILE__, __LINE__,
            "Arm26 activation "+muscName+" with bounds failed.");

        CHECK_STORAGE_AGAINST_STANDARD(forces2, stdForces2,
            std::vector<double>(6, forceTol),
            __FILE__,  __LINE__,
            "Arm26 forces "+muscName+" with bounds failed.");

        cout << resultsDir << ": testArm26 with bounds passed" << endl;
        cout << "=============================================================\n" << endl;
    }
}


TEST_CASE("testArm26")
{
    // Muscle model formulations to test, with per-model tolerances for the
    // difference between the current model and the 'standard' solution (which
    // was closest to the Thelen2003Muscle_Deprecated formulation).
    const std::vector<std::string> muscleModelNames = {
        "Thelen2003Muscle_Deprecated",
        "Thelen2003Muscle",
        "Millard2012EquilibriumMuscle",
        //"Millard2012AccelerationMuscle",
    };
    const double actTols[3] = {0.005, 0.025, 0.04};
    const double forceTols[3] = {1, 4, 5};

    for (size_t i = 0; i < muscleModelNames.size(); ++i) {
        DYNAMIC_SECTION(muscleModelNames[i]) {
            testArm26(muscleModelNames[i], actTols[i], forceTols[i]);
        }
    }
}


// TEST_CASE("testModelWithPassiveForces") {
//     AnalyzeTool analyze("staticoptimization_spring_Setup.xml");
//     analyze.run();
//     std::string resultsDir("ResultsSO_spring");
//     Storage activations(resultsDir + "/walk_subject01_ankle_spring_StaticOptimization_activation.sto");
//     Storage stdActivations("std_walk_subject01_ankle_spring_StaticOptimization_activation.sto");

//     Storage forces(resultsDir + "/walk_subject01_ankle_spring_StaticOptimization_force.sto");
//     Storage stdForces("std_walk_subject01_ankle_spring_StaticOptimization_force.sto");

//     CHECK_STORAGE_AGAINST_STANDARD(activations, stdActivations,
//         std::vector<double>(28, 0.025),
//         __FILE__, __LINE__,
//         "ModelWithPassiveForces activations failed");

//     CHECK_STORAGE_AGAINST_STANDARD(forces, stdForces,
//         std::vector<double>(48, 3),
//         __FILE__, __LINE__,
//         "ModelWithPassiveForces forces failed.");
//     cout << resultsDir << ": test ModelWithPassiveForces passed." << endl;

// }


// TEST_CASE("testLapackErrorDLASD4")  {
//     // With OpenSim 3.2 64bit, the 64 bit lapack library (in Simbody 3.3.1)
//     // crashes with an error[1] if there are not enough actuators (or under
//     // other similar circumstances). With Simbody 3.5.2, the 64 bit Windows lapack
//     // libraries are updated and do not have this bug.
//     // This test ensures that the bug is gone. We only expect this test to fail
//     // on Windows 64 bit build using Simbody 3.3.1 or earlier.
//     // What this function should do is print SimTK exceptions about not being
//     // able to satisfy constraints, but nothing should crash and
//     // no exceptions should reach this function.
//     //
//     // The error is caused by using unfiltered inverse kinematics as input.
//     //
//     // [1] The error was:
//     //     "** On entry to DLASD4 parameter number -1 had an illegal value"
//     AnalyzeTool analyze("subject01_Setup_StaticOptimization.xml");
//     analyze.setResultsDir("Results_subject01_StaticOptimization_LapackError");
//     analyze.run();
// }

// TEST_CASE("testRelativePathInExternalLoads") {
//     // Ensure that we can handle relative paths in the ExternalLoads XML file.
//     // It's important that we do not run with the current working directory as
//     // the location of Setup_SO.xml.
//     AnalyzeTool analyze("UsingRelativePaths/Setup_SO.xml");
//     analyze.setResultsDir("Results_UsingRelativePaths");
//     analyze.run();
// }

// TEST_CASE("testArm26DisabledMuscles") {
//     const auto arm26Dir = opensim_tests_resources_directory() / "models" / "Arm26";
//     AnalyzeTool analyze((arm26Dir / "arm26_Setup_StaticOptimization.xml").string());
//     analyze.setResultsDir((std::filesystem::current_path()
//             / "Results_arm26_StaticOptimization_Disabled").string());
//     Model& model=analyze.getModel();
//     model.updComponent<Actuator>("/forceset/TRIlat").set_appliesForce(false);
//     model.updComponent<Actuator>("/forceset/TRImed").set_appliesForce(false);
//     analyze.run();
//     Storage activations(analyze.getResultsDir() + "/arm26_StaticOptimization_activation.sto");
//     ASSERT_EQUAL(activations.getColumnLabels().size(), 5);
//     ASSERT_EQUAL(activations.getColumnLabels().findIndex("TRIlat"), -1);
//     ASSERT_EQUAL(activations.getColumnLabels().findIndex("TRImed"), -1);
//     Storage forces(analyze.getResultsDir() + "/arm26_StaticOptimization_force.sto");
//     ASSERT_EQUAL(forces.getColumnLabels().size(), 5);
//     ASSERT_EQUAL(forces.getColumnLabels().findIndex("TRIlat"), -1);
//     ASSERT_EQUAL(forces.getColumnLabels().findIndex("TRImed"), -1);

//     // Ensure time values in the stored state derivatives match the times in the state
//     StaticOptimization& statOpt =(StaticOptimization&)model.getAnalysisSet().get("StaticOptimization");
//     Storage* statesDerivativeStore = statOpt.getStatesDerivativeStore();
//     const Storage* statesStore = statOpt._statesStore;
//     Array<double> time;
//     Array<double> time_d;
//     int nt = statesStore->getTimeColumn(time);
//     int nt_d = statesDerivativeStore->getTimeColumn(time_d);
//     ASSERT_EQUAL(nt, nt_d);
//     ASSERT_EQUAL<Array<double>>(time, time_d, std::numeric_limits<double>::epsilon());
// }
