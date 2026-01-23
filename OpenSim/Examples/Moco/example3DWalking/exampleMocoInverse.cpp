/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleMocoInverse.cpp                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2023 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia, Nicholas Bianco                             *
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
/// from electromyography data for a subset of muscles. The third example
/// extracts muscle synergies from the muscle excitaitons from the first example
/// and uses them to solve the inverse problem using SynergyControllers.
///
/// See the README.txt next to this file for more information.

#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Common/Adapters.h>
#include <OpenSim/Simulation/Control/SynergyController.h>
#include <OpenSim/Moco/osimMoco.h>

using namespace OpenSim;

/// Solve the basic muscle redundancy problem with MocoInverse.
void solveMocoInverse() {

    // Construct the MocoInverse tool.
    MocoInverse inverse;
    inverse.setName("example3DWalking_MocoInverse");

    // Construct a ModelProcessor and set it on the tool. The default
    // muscles in the model are replaced with optimization-friendly
    // DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
    // parameters.
    ModelProcessor modelProcessor("subject_walk_scaled.osim");
    modelProcessor.append(ModOpAddExternalLoads("grf_walk.xml"));
    // Replace the PinJoints representing the model's toes with WeldJoints.
    modelProcessor.append(ModOpReplaceJointsWithWelds({"mtp_r", "mtp_l"}));
    // Add CoordinateActuators to the pelvis coordinates. 
    modelProcessor.append(ModOpAddResiduals(250.0, 50.0, 1.0));
    modelProcessor.append(ModOpIgnoreTendonCompliance());
    modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
    // Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
    // Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
    // Use a function-based representation for the muscle paths. This is
    // recommended to speed up convergence, but if you would like to use
    // the original GeometryPath muscle wrapping instead, simply comment out
    // this line. To learn how to create a set of function-based paths for
    // your model, see the example 'examplePolynomialPathFitter.py/.m'.
    modelProcessor.append(ModOpReplacePathsWithFunctionBasedPaths(
            "subject_walk_scaled_FunctionBasedPathSet.xml"));
    modelProcessor.append(ModOpAddReserves(1.0));
    inverse.setModel(modelProcessor);

    // Construct a TableProcessor of the coordinate data and pass it to the
    // inverse tool. TableProcessors can be used in the same way as
    // ModelProcessors by appending TableOperators to modify the base table.
    // A TableProcessor with no operators, as we have here, simply returns the
    // base table.
    inverse.setKinematics(TableProcessor("coordinates.sto"));

    // Initial time, final time, and mesh interval.
    inverse.set_initial_time(0.48);
    inverse.set_final_time(1.61);
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
/// subset of muscles.
void solveMocoInverseWithEMG() {

    // This initial block of code is identical to the code above.
    MocoInverse inverse;
    inverse.setName("example3DWalking_MocoInverseWithEMG");
    ModelProcessor modelProcessor("subject_walk_scaled.osim");
    modelProcessor.append(ModOpAddExternalLoads("grf_walk.xml"));
    // Replace the PinJoints representing the model's toes with WeldJoints.
    modelProcessor.append(ModOpReplaceJointsWithWelds({"mtp_r", "mtp_l"}));
    modelProcessor.append(ModOpAddResiduals(250.0, 50.0, 1.0));
    modelProcessor.append(ModOpIgnoreTendonCompliance());
    modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
    // Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
    // Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
    // Use a function-based representation for the muscle paths. This is
    // recommended to speed up convergence, but if you would like to use
    // the original GeometryPath muscle wrapping instead, simply comment out
    // this line. To learn how to create a set of function-based paths for
    // your model, see the example 'examplePolynomialPathFitter.py/.m'.
    modelProcessor.append(ModOpReplacePathsWithFunctionBasedPaths(
            "subject_walk_scaled_FunctionBasedPathSet.xml"));
    modelProcessor.append(ModOpAddReserves(1.0));
    inverse.setModel(modelProcessor);
    inverse.setKinematics(TableProcessor("coordinates.sto"));
    inverse.set_initial_time(0.48);
    inverse.set_final_time(1.61);
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

/// This problem extracts muscle synergies from the muscle excitations from the
/// first example and uses them to solve the inverse problem using
/// SynergyControllers.
void solveMocoInverseWithSynergies(int numSynergies = 5) {

    // Construct the base model using a ModelProcessor as in the previous
    // examples, with the exception that we ignore activation dynamics to
    // simplify the problem given the muscle synergy controllers.
    ModelProcessor modelProcessor("subject_walk_scaled.osim");
    modelProcessor.append(ModOpAddExternalLoads("grf_walk.xml"));
    // Replace the PinJoints representing the model's toes with WeldJoints.
    modelProcessor.append(ModOpReplaceJointsWithWelds({"mtp_r", "mtp_l"}));
    modelProcessor.append(ModOpAddResiduals(250.0, 50.0, 1.0));
    modelProcessor.append(ModOpIgnoreTendonCompliance());
    modelProcessor.append(ModOpIgnoreActivationDynamics());
    modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
    modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
    modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
    modelProcessor.append(ModOpReplacePathsWithFunctionBasedPaths(
            "subject_walk_scaled_FunctionBasedPathSet.xml"));
    modelProcessor.append(ModOpAddReserves(1.0));
    Model model = modelProcessor.process();

    // Load the solution from solveMocoInverse() to extract the muscle
    // control variable names and excitations for the left and right legs.
    MocoTrajectory prevSolution("example3DWalking_MocoInverse_solution.sto");
    std::vector<std::string> leftControlNames;
    std::vector<std::string> rightControlNames;
    for (const auto& muscle : model.getComponentList<Muscle>()) {
        const auto& name = muscle.getName();
        if (name.compare(name.length() - 2, 2, "_l") == 0) {
            leftControlNames.push_back(muscle.getAbsolutePathString());
        } else if (name.compare(name.length() - 2, 2, "_r") == 0) {
            rightControlNames.push_back(muscle.getAbsolutePathString());
        }
    }

    TimeSeriesTable controls = prevSolution.exportToControlsTable();
    TimeSeriesTable leftControls(controls.getIndependentColumn());
    TimeSeriesTable rightControls(controls.getIndependentColumn());
    for (const auto& name : leftControlNames) {
        leftControls.appendColumn(name, controls.getDependentColumn(name));
    }
    for (const auto& name : rightControlNames) {
        rightControls.appendColumn(name, controls.getDependentColumn(name));
    }

    // SynergyController
    // -----------------
    // SynergyController defines the controls for actuators connected to the 
    // controller using a linear combination of time-varying synergy control 
    // signals (i.e., "synergy excitations") and a set of vectors containing 
    // weights for each actuator representing the contribution of each synergy
    // excitation to the total control signal for that actuator 
    // (i.e., "synergy vectors").
    //
    // If 'N' is the number of time points in the trajectory, 'M' is the number
    // of actuators connected to the controller, and 'K' is the number of   
    // synergies in the controller, then:
    // - The synergy excitations are a matrix 'W' of size N x K.
    // - The synergy vectors are a matrix 'H' of size K x M.
    // - The controls for the actuators are computed A = W * H.
    //  
    // SynergyController is a concrete implementation of InputController, which
    // means that it uses Input control signals as the synergy excitations.
    // Moco automatically detects InputController%s in a model provided to a
    // MocoProblem and adds continuous variables to the optimization problem
    // for each Input control signal. The variable names are based on the path
    // to the controller appended with the Input control labels (e.g.,
    // "/path/to/my_synergy_controller/synergy_excitation_0").

    // Use non-negative matrix factorization (NNMF) to extract a set of muscle
    // synergies for each leg.
    int maxIterations = 100;
    double tolerance = 1e-6;

    SimTK::Matrix Wl;
    SimTK::Matrix Hl;
    factorizeMatrixNonNegative(leftControls.getMatrix(), numSynergies, 
            maxIterations, tolerance, Wl, Hl);

    SimTK::Matrix Wr;
    SimTK::Matrix Hr;
    factorizeMatrixNonNegative(rightControls.getMatrix(), numSynergies, 
            maxIterations, tolerance, Wr, Hr);

    // Add a SynergyController for the left leg to the model.
    auto* leftController = new SynergyController();
    leftController->setName("synergy_controller_left_leg");
    // The number of actuators connected to the controller defines the number of
    // weights in each synergy vector expected by the controller.
    for (const auto& name : leftControlNames) {
        leftController->addActuator(model.getComponent<Muscle>(name));
    }
    // Adding a synergy vector increases the number of synergies in the
    // controller by one. This means that the number of Input control 
    // signals expected by the controller is also increased by one.
    for (int i = 0; i < numSynergies; ++i) {    
        leftController->addSynergyVector(Hl.row(i).transpose().getAsVector());
    }
    model.addController(leftController);

    // Add a SynergyController for the right leg to the model.
    auto* rightController = new SynergyController();
    rightController->setName("synergy_controller_right_leg");
    for (const auto& name : rightControlNames) {
        rightController->addActuator(model.getComponent<Muscle>(name));
    }
    for (int i = 0; i < numSynergies; ++i) {
        rightController->addSynergyVector(Hr.row(i).transpose().getAsVector());
    }
    model.addController(rightController);
    model.finalizeConnections();
    model.initSystem();

    // Construct the MocoInverse tool.
    MocoInverse inverse;
    inverse.setName("example3DWalking_MocoInverseWithSynergies");
    inverse.setModel(ModelProcessor(model));
    inverse.setKinematics(TableProcessor("coordinates.sto"));
    inverse.set_initial_time(0.48);
    inverse.set_final_time(1.61);
    inverse.set_mesh_interval(0.02);
    inverse.set_kinematics_allow_extra_columns(true);

    // Initialize the MocoInverse study and set the control bounds for the
    // muscle synergies excitations. 'setInputControlInfo()' is equivalent to
    // 'setControlInfo()', but reserved for Input control variables. Note that 
    // while we make a distinction between "control" variables and 
    // "Input control" variables in the API, the optimal control problem
    // constructed by Moco treats them both as algebraic variables.
    MocoStudy study = inverse.initialize();
    auto& problem = study.updProblem();

    // We will also increase the weight on the synergy excitations in the 
    // control effort cost term. MocoControlGoal, and other MocoGoals, that 
    // depend on control variables have options configuring cost terms with
    // Input control values.
    auto& effort = dynamic_cast<MocoControlGoal&>(
            problem.updGoal("excitation_effort"));
    for (int i = 0; i < numSynergies; ++i) {
        std::string nameLeft = fmt::format("/controllerset/"
                "synergy_controller_left_leg/synergy_excitation_{}", i);
        problem.setInputControlInfo(nameLeft, {0, 1.0});
        effort.setWeightForControl(nameLeft, 10);

        std::string nameRight = fmt::format("/controllerset/"
                "synergy_controller_right_leg/synergy_excitation_{}", i);
        problem.setInputControlInfo(nameRight, {0, 1.0});
        effort.setWeightForControl(nameRight, 10);
    }

    // Solve!
    MocoSolution solution = study.solve();

    // This function computes the model control values from the
    // SynergyControllers in the model and inserts them into the solution.
    solution.generateControlsFromModelControllers(model);

    // Add the multibody states into the solutions so we can visualize the
    // trajectory or utilize the plotting utilities.
    TimeSeriesTable coordinateValues = prevSolution.exportToValuesTable();
    TimeSeriesTable coordinateSpeeds = prevSolution.exportToSpeedsTable();
    solution.insertStatesTrajectory(coordinateValues);
    solution.insertStatesTrajectory(coordinateSpeeds);

    // Write the solution to a Storage file.
    solution.write(fmt::format("example3DWalking_MocoInverseWith{}Synergies_"
            "solution.sto", numSynergies));
}

int main() {

    // Solve the basic muscle redundancy problem with MocoInverse.
    solveMocoInverse();

    // This problem penalizes the deviation from electromyography data for a
    // subset of muscles.
    solveMocoInverseWithEMG();

    /// This problem extracts muscle synergies from the muscle excitations from
    /// the first example and uses them to solve the inverse problem using
    /// SynergyControllers.
    int numSynergies = 5;
    solveMocoInverseWithSynergies(numSynergies);

    // If you installed the Moco python package, you can compare both solutions
    // using the following command:
    //      opensim-moco-generate-report subject_walk_scaled.osim
    //          example3DWalking_MocoInverse_solution.sto --bilateral
    //          --ref_files example3DWalking_MocoInverseWithEMG_solution.sto
    //                      controls_reference.sto

    return EXIT_SUCCESS;
}
