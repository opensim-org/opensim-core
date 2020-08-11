/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleMocoInverse.cpp                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/// This example shows how to use the MocoInverse tool to exactly prescribe a
/// motion and estimate muscle behavior for walking. The first example does not
/// rely on electromyography data, while the second example penalizes deviation
/// from electromyography data for a subset of muscles.
///
/// See the README.txt next to this file for more information.

#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Common/Adapters.h>
#include <OpenSim/Moco/osimMoco.h>

using namespace OpenSim;

/// This problem solves in about 5 minutes.
void solveMocoInverse() {

    // Construct the MocoInverse tool.
    MocoInverse inverse;
    inverse.setName("example3DWalking_MocoInverse");

    // Construct a ModelProcessor and set it on the tool. The default
    // muscles in the model are replaced with optimization-friendly
    // DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
    // parameters.
    ModelProcessor modelProcessor("subject_walk_armless.osim");
    modelProcessor.append(ModOpAddExternalLoads("grf_walk.xml"));
    modelProcessor.append(ModOpIgnoreTendonCompliance());
    modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
    // Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
    // Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
    modelProcessor.append(ModOpAddReserves(1.0));
    inverse.setModel(modelProcessor);

    // Construct a TableProcessor of the coordinate data and pass it to the
    // inverse tool. TableProcessors can be used in the same way as
    // ModelProcessors by appending TableOperators to modify the base table.
    // A TableProcessor with no operators, as we have here, simply returns the
    // base table.
    inverse.setKinematics(TableProcessor("coordinates.sto"));

    // Initial time, final time, and mesh interval.
    inverse.set_initial_time(0.81);
    inverse.set_final_time(1.79);
    inverse.set_mesh_interval(0.02);

    // By default, Moco gives an error if the kinematics contains extra columns.
    // Here, we tell Moco to allow (and ignore) those extra columns.
    inverse.set_kinematics_allow_extra_columns(true);

    // Solve the problem and write the solution to a Storage file.
    MocoInverseSolution solution = inverse.solve();
    solution.getMocoSolution().write(
            "example3DWalking_MocoInverse_solution.sto");

}

/// This problem penalizes the deviation from electromyography data for a
/// subset of muscles, and solves in about 30 minutes.
void solveMocoInverseWithEMG() {

    // This initial block of code is identical to the code above.
    MocoInverse inverse;
    inverse.setName("example3DWalking_MocoInverseWithEMG");
    ModelProcessor modelProcessor("subject_walk_armless.osim");
    modelProcessor.append(ModOpAddExternalLoads("grf_walk.xml"));
    modelProcessor.append(ModOpIgnoreTendonCompliance());
    modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
    // Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
    // Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
    modelProcessor.append(ModOpAddReserves(1.0));
    inverse.setModel(modelProcessor);
    inverse.setKinematics(TableProcessor("coordinates.sto"));
    inverse.set_initial_time(0.81);
    inverse.set_final_time(1.79);
    inverse.set_mesh_interval(0.02);
    inverse.set_kinematics_allow_extra_columns(true);

    MocoStudy study = inverse.initialize();
    MocoProblem& problem = study.updProblem();

    // Add electromyography tracking.
    auto* tracking = problem.addGoal<MocoControlTrackingGoal>("emg_tracking");
    tracking->setWeight(50.0);
    // Each column in electromyography.sto is normalized so the maximum value in
    // each column is 1.0.
    TimeSeriesTable controlsRef("electromyography.sto");
    // Scale the tracked muscle activity based on peak levels from
    // "Gait Analysis: Normal and Pathological Function" by
    // Perry and Burnfield, 2010 (digitized by Carmichael Ong).
    controlsRef.updDependentColumn("soleus") *= 0.77;
    controlsRef.updDependentColumn("gastrocnemius") *= 0.87;
    controlsRef.updDependentColumn("tibialis_anterior") *= 0.37;
    tracking->setReference(controlsRef);
    // Associate actuators in the model with columns in electromyography.sto.
    tracking->setReferenceLabel("/forceset/soleus_r", "soleus");
    tracking->setReferenceLabel("/forceset/gasmed_r", "gastrocnemius");
    tracking->setReferenceLabel("/forceset/gaslat_r", "gastrocnemius");
    tracking->setReferenceLabel("/forceset/tibant_r", "tibialis_anterior");

    // Solve the problem and write the solution to a Storage file.
    MocoSolution solution = study.solve();
    solution.write("example3DWalking_MocoInverseWithEMG_solution.sto");

    // Write the reference data in a way that's easy to compare to the solution.
    controlsRef.removeColumn("medial_hamstrings");
    controlsRef.removeColumn("biceps_femoris");
    controlsRef.removeColumn("vastus_lateralis");
    controlsRef.removeColumn("vastus_medius");
    controlsRef.removeColumn("rectus_femoris");
    controlsRef.removeColumn("gluteus_maximus");
    controlsRef.removeColumn("gluteus_medius");
    controlsRef.setColumnLabels({"/forceset/soleus_r", "/forceset/gasmed_r",
                                 "/forceset/tibant_r"});
    controlsRef.appendColumn("/forceset/gaslat_r",
            controlsRef.getDependentColumn("/forceset/gasmed_r"));
    STOFileAdapter::write(controlsRef, "controls_reference.sto");
}

int main() {

    solveMocoInverse();

    solveMocoInverseWithEMG();

    // If you installed the Moco python package, you can compare both solutions
    // using the following command:
    //      opensim-moco-generate-report subject_walk_armless.osim
    //          example3DWalking_MocoInverse_solution.sto --bilateral
    //          --ref_files example3DWalking_MocoInverseWithEMG_solution.sto
    //                      controls_reference.sto

    return EXIT_SUCCESS;
}
