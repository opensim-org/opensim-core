/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxMocoTrack.cpp                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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

#include <algorithm>
#include <Moco/osimMoco.h>

#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using SimTK::Infinity;

class DecoratedCoordinateActuator : public CoordinateActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            DecoratedCoordinateActuator, CoordinateActuator);
public:
    DecoratedCoordinateActuator() = default;

protected:
    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
            const SimTK::State& s,
            SimTK::Array_<SimTK::DecorativeGeometry>& geometryArray)
            const override {
        Super::generateDecorations(fixed, hints, s, geometryArray);

        const auto& frame = getCoordinate()->getJoint().getParentFrame();
        if (!fixed && (s.getSystemStage() >= SimTK::Stage::Dynamics)) {
            double act = getActuation(s);

            SimTK::DecorativeSphere sphere(act*0.1);
            sphere.setTransform(frame.getTransformInGround(s));
            geometryArray.push_back(sphere);
        }
    }
};

void addCoordinateActuator(Model& model, std::string coordName, 
        double optimalForce, double min = -Infinity,
        double max = Infinity) {

    auto& coordSet = model.updCoordinateSet();

    auto* actu = new CoordinateActuator();
    actu->setName("tau_" + coordName);
    actu->setCoordinate(&coordSet.get(coordName));
    actu->setOptimalForce(optimalForce);
    actu->setMinControl(min);
    actu->setMaxControl(max);
    model.addComponent(actu);
}

void addDecorativeCoordinateActuator(Model& model, std::string coordName,
        double optimalForce, double min = -Infinity, double max = Infinity) {

    auto& coordSet = model.updCoordinateSet();

    auto* actu = new DecorativeCoordinateActuator();
    actu->setName("tau_" + coordName);
    actu->setCoordinate(&coordSet.get(coordName));
    actu->setOptimalForce(optimalForce);
    actu->setMinControl(min);
    actu->setMaxControl(max);
    model.addComponent(actu);
}

void transformReactionToBodyFrame(const MocoStudy& study, 
        const MocoTrajectory& iterate, 
        TimeSeriesTable_<SimTK::SpatialVec>& reactionTable) {
    auto model = study.getProblem().createRep().getModelBase();
    model.initSystem();
    const auto& ground = model.getGround();
    auto statesTraj = iterate.exportToStatesTrajectory(study.getProblem());
    assert(statesTraj.getSize() == reactionTable.getNumRows());

    for (int irow = 0; irow < reactionTable.getNumRows(); ++irow) {
        auto& row = reactionTable.updRowAtIndex(irow);
        for (int ielt = 0; ielt < row.size(); ++ielt) {

            const auto& state = statesTraj.get(irow);
            const auto& label = reactionTable.getColumnLabel(ielt);
            std::string frameName;
            if (label.find("walker_knee_l") != std::string::npos) {
                frameName = "/bodyset/tibia_l";
            } else if (label.find("walker_knee_r") != std::string::npos) {
                frameName = "/bodyset/tibia_r";
            }
            const auto& frame = model.getComponent<Frame>(frameName);

            const auto& elt = row.getElt(0, ielt);
            model.realizeAcceleration(state);
            SimTK::Vec3 moment = ground.expressVectorInAnotherFrame(state,
                elt[0], frame);
            SimTK::Vec3 force = ground.expressVectorInAnotherFrame(state,
                elt[1], frame);

            SimTK::SpatialVec newElt(moment, force);
            row.updElt(0, ielt) = newElt;
        }
    }
}

TimeSeriesTable getTransformTrajectories(Model model,
        const StatesTrajectory& statesTraj,
        std::vector<std::string> compPaths) {

    // Create independent time vector and construct table.
    std::vector<double> indCol;
    for (const auto& state : statesTraj) {
        indCol.push_back((double)state.getTime());
    }
    TimeSeriesTable table(indCol);

    // Append columns.
    model.initSystem();
    SimTK::Matrix mat((int)indCol.size(), 7*(int)compPaths.size());
    std::vector<std::string> colLabels;
    for (int row = 0; row < statesTraj.getSize(); ++row) {
        auto state = statesTraj.get(row);
        model.getSystem().prescribe(state);
        model.realizePosition(state);

        int col = 0;
        for (const auto& compPath : compPaths) {
            SimTK::Transform transform =
                model.getComponent(compPath)
                .getOutputValue<SimTK::Transform>(state, "transform");

            // Rotations.
            const auto& R = transform.R();
            auto e = R.convertRotationToQuaternion();
            for (int i = 0; i < 4; ++i) {
                mat.updElt(row, col++) = e[i];
                if (!row) {
                    colLabels.push_back(format("%s/transform_e%i",
                        compPath, i + 1));
                }
            }
            // Position vector.
            const auto& p = transform.p();
            for (int i = 0; i < 3; ++i) {
                mat.updElt(row, col++) = p(i);
                if (!row) {
                    colLabels.push_back(format("%s/transform_p%i", compPath,
                        i + 1));
                }
            }
        }
    }

    table.updMatrix() = mat;
    table.setColumnLabels(colLabels);

    return table;
}

TimeSeriesTable computeTransformErrors(Model model, 
        std::string solution1, std::string solution2, 
        std::vector<std::string> compPaths) {

    auto traj1 = StatesTrajectory::createFromStatesStorage(model, 
        Storage(solution1), true, true);
    auto traj2 = StatesTrajectory::createFromStatesStorage(model,
        Storage(solution2), true, true);

    auto table1 = getTransformTrajectories(model, traj1, compPaths);
    auto table2 = getTransformTrajectories(model, traj2, compPaths);
    STOFileAdapter::write(table1,
        solution1.replace(solution1.end() - 4, solution1.end(),
            "_transforms.sto"));
    STOFileAdapter::write(table2,
        solution2.replace(solution2.end() - 4, solution2.end(),
            "_transforms.sto"));

    auto splines1 = GCVSplineSet(table1);
    auto splines2 = GCVSplineSet(table2);

    auto time = table1.getIndependentColumn();

    TimeSeriesTable errorTable;
    std::vector<std::string> errorLabels;

    for (int i = 0; i < (int)time.size(); ++i) {
        SimTK::RowVector error(4*(int)compPaths.size(), 0.0);
        SimTK::Vector timeVec(1, time[i]);

        for (int j = 0; j < compPaths.size(); ++j) {

            const SimTK::Quaternion e1(
                splines1[7*j].calcValue(timeVec),
                splines1[7*j + 1].calcValue(timeVec),
                splines1[7*j + 2].calcValue(timeVec),
                splines1[7*j + 3].calcValue(timeVec));
            const SimTK::Rotation R_GS1(e1);

            const SimTK::Quaternion e2(
                splines2[7*j].calcValue(timeVec),
                splines2[7*j + 1].calcValue(timeVec),
                splines2[7*j + 2].calcValue(timeVec),
                splines2[7*j + 3].calcValue(timeVec));
            const SimTK::Rotation R_GS2(e2);

            const SimTK::Rotation R_S1S2 = ~R_GS1*R_GS2;
            const SimTK::Vec4 aa_S1S2 = R_S1S2.convertRotationToAngleAxis();
            error[4*j] = aa_S1S2[0];

            if (!i) {
                errorLabels.push_back(format("%s_Rerr", compPaths[j]));
            }

            for (int k = 0; k < 3; ++k) {
                error[4*j + k + 1] = (splines2[7*j + 4 + k].calcValue(timeVec)
                    - splines1[7*j + 4 + k].calcValue(timeVec));
                if (!i) {
                    errorLabels.push_back(format("%s_p%ierr", compPaths[j], 
                        k + 1));
                }
            }
        }
        errorTable.appendRow(time[i], error);
    }
    errorTable.setColumnLabels(errorLabels);

    return errorTable;
}

Model createModel(bool removeMuscles = false, bool addAnkleExo = false) {

    std::string model_file = "subject_walk_armless_foot_markers.osim";
    Model model(model_file);
    if (removeMuscles) {
        model.updForceSet().clearAndDestroy();
    } else {
        for (auto& musc : model.updComponentList<Muscle>()) {
            musc.set_ignore_tendon_compliance(true);
            musc.set_ignore_activation_dynamics(false);
        }
    }
    model.initSystem();

    // weld joints w/ locked coordinates
    ModelFactory::replaceJointWithWeldJoint(model, "mtp_l");
    ModelFactory::replaceJointWithWeldJoint(model, "mtp_r");
    ModelFactory::replaceJointWithWeldJoint(model, "subtalar_l");
    ModelFactory::replaceJointWithWeldJoint(model, "subtalar_r");
    // coordinate actuators
    //addCoordinateActuator(model, "lumbar_bending", 1);
    //addCoordinateActuator(model, "lumbar_extension", 1);
    //addCoordinateActuator(model, "lumbar_rotation", 1);
    addCoordinateActuator(model, "pelvis_tilt", 1);
    addCoordinateActuator(model, "pelvis_list", 1);
    addCoordinateActuator(model, "pelvis_rotation", 1);
    addCoordinateActuator(model, "pelvis_tx", 1);
    addCoordinateActuator(model, "pelvis_ty", 1);
    addCoordinateActuator(model, "pelvis_tz", 1);
    if (removeMuscles) {
        addCoordinateActuator(model, "hip_adduction_l", 100);
        addCoordinateActuator(model, "hip_adduction_r", 100);
        addCoordinateActuator(model, "hip_flexion_l", 100);
        addCoordinateActuator(model, "hip_flexion_r", 100);
        addCoordinateActuator(model, "hip_rotation_l", 100);
        addCoordinateActuator(model, "hip_rotation_r", 100);
        addCoordinateActuator(model, "knee_angle_l", 100);
        addCoordinateActuator(model, "knee_angle_r", 100);
        addCoordinateActuator(model, "ankle_angle_l", 250);
        addCoordinateActuator(model, "ankle_angle_r", 250);
        addCoordinateActuator(model, "subtalar_angle_r", 100);
        addCoordinateActuator(model, "subtalar_angle_l", 100);
    }

    if (addAnkleExo) {
        addCoordinateActuator(model, "ankle_angle_l", 250, -1, 0);
        addCoordinateActuator(model, "ankle_angle_r", 250, -1, 0); 
    }

    if (removeMuscles) {
        model.print(model_file.replace(
                model_file.end()-5, model_file.end(), "_updated.osim"));
    } else {
        model.print(model_file.replace(
                model_file.end()-5, model_file.end(), 
                "_updated_muscles.osim"));
    }

    return model;
}

void smoothSolutionControls(Model model, std::string statesFile, 
        const std::string& guessFile) {

    MocoTrack track;
    track.setName("smoothed");
    ModelProcessor modelProcessor = ModelProcessor(model) |
                                    ModOpAddExternalLoads("grf_walk.xml");
    track.setModel(model);
    TableProcessor tableProcessor = TableProcessor(statesFile) | 
                                    TabOpLowPassFilter(6);
    track.set_states_reference(tableProcessor);
    track.set_guess_file(guessFile);
    track.set_initial_time(0.812);
    track.set_final_time(1.648);
    track.set_control_effort_weight(0.05);

    MocoStudy study = track.initialize();

    auto& solver = study.updSolver<MocoCasADiSolver>();
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_convergence_tolerance(1e-4);

    MocoProblem problem = study.getProblem();

    MocoSolution solution = study.solve().unseal();
    solution.write(statesFile.replace(statesFile.end()-4, statesFile.end(),
        "_smoothed.sto"));
    study.visualize(solution);

    TimeSeriesTable_<SimTK::SpatialVec> reactionTable =
        analyze<SimTK::SpatialVec>(model, solution,
        {"/jointset/walker_knee_l/reaction_on_child",
            "/jointset/walker_knee_r/reaction_on_child"});

    transformReactionToBodyFrame(study, solution, reactionTable);
    STOFileAdapter::write(reactionTable.flatten(),
        statesFile.replace(statesFile.end() - 4, statesFile.end(),
            "_reactions.sto"));
}

MocoSolution runBaselineProblem(bool removeMuscles, double controlWeight = 0.1,
        std::string guessFile = "") {
    
    Model model = createModel(removeMuscles);
    if (!removeMuscles) {
        DeGrooteFregly2016Muscle::replaceMuscles(model);
        for (auto& musc : model.updComponentList<DeGrooteFregly2016Muscle>()) {
            musc.set_ignore_passive_fiber_force(true);
            musc.set_active_force_width_scale(1.5);
            //musc.set_max_isometric_force(2 * musc.get_max_isometric_force());
        }
    }

    // Baseline tracking problem.
    // --------------------------
    MocoTrack track;
    track.setName("baseline");
    ModelProcessor modelProcessor = ModelProcessor(model) |
                                    ModOpAddExternalLoads("grf_walk.xml");
    track.setModel(modelProcessor);
    TableProcessor tableProcessor =
            TableProcessor("coordinates_rra_adjusted.sto");
    track.set_states_reference(tableProcessor);
    track.setMarkersReferenceFromTRC("marker_trajectories.trc");
    track.set_track_reference_position_derivatives(true);
    track.set_initial_time(0.81);
    track.set_final_time(1.65);
    track.set_control_effort_weight(controlWeight);
    //track.set_states_global_tracking_weight(10);
    track.set_scale_state_weights_with_range(true);
    track.set_allow_unused_references(true);
    track.set_mesh_interval(0.02);

    track.print("sandboxMocoTrack_baseline_muscles_MocoTrack.omoco");

    MocoStudy study = track.initialize();

    auto& solver = study.updSolver<MocoCasADiSolver>();
    solver.set_optim_constraint_tolerance(1e-3);
    solver.set_optim_convergence_tolerance(1e-3);
    if (guessFile != "") {
        solver.setGuessFile(guessFile);
    }

    study.print("sandboxMocoTrack_baseline_muscles_MocoStudy.omoco");
    MocoSolution solution = study.solve().unseal();
    std::string filename;
    if (removeMuscles) {
        filename = "sandboxMocoTrack_solution_baseline.sto";
    } else {
        filename = "sandboxMocoTrack_solution_baseline_muscles.sto";
    }
    solution.write(filename);
    study.visualize(solution);

    //TimeSeriesTable_<SimTK::SpatialVec> reactionTable =
    //        analyze<SimTK::SpatialVec>(model, solution,
    //            {"/jointset/walker_knee_l/reaction_on_child",
    //             "/jointset/walker_knee_r/reaction_on_child"});

    //transformReactionToBodyFrame(study, solution, reactionTable);
    //STOFileAdapter::write(reactionTable.flatten(),
    //    filename.replace(filename.end()-4, filename.end(), 
    //        "_reactions.sto"));
    //
    return solution;
}

MocoSolution runExoskeletonProblemMarkers(double controlWeight,
        std::string trackedTrajectoryFile) {

    Model model = createModel(false, true);
    DeGrooteFregly2016Muscle::replaceMuscles(model);
    for (auto& musc : model.updComponentList<DeGrooteFregly2016Muscle>()) {
        musc.set_ignore_passive_fiber_force(true);
        musc.set_active_force_width_scale(1.5);
    }
  

    // Baseline tracking problem.
    // --------------------------
    MocoTrack track;
    track.setName("baseline");
    ModelProcessor modelProcessor =
            ModelProcessor(model) | ModOpAddExternalLoads("grf_walk.xml");
    track.setModel(modelProcessor);
    track.set_states_reference(TableProcessor(trackedTrajectoryFile));
    track.setMarkersReferenceFromTRC("marker_trajectories.trc");
    track.set_track_reference_position_derivatives(true);
    track.set_initial_time(0.81);
    track.set_final_time(1.65);
    track.set_control_effort_weight(controlWeight);
    track.set_scale_state_weights_with_range(true);
    track.set_allow_unused_references(true);
    track.set_mesh_interval(0.02);

    track.print("sandboxMocoTrack_exoskeleton_markers_MocoTrack.omoco");

    MocoStudy study = track.initialize();

    auto& effort = dynamic_cast<MocoControlCost&>(
            study.updProblem().updCost("control_effort"));
    effort.setWeight("/tau_ankle_angle_r", 0.0);
    effort.setWeight("/tau_ankle_angle_l", 0.0);

    auto& solver = study.updSolver<MocoCasADiSolver>();
    solver.set_optim_constraint_tolerance(1e-3);
    solver.set_optim_convergence_tolerance(1e-3);
    MocoTrajectory guess(trackedTrajectoryFile);
    auto time = guess.getTime();
    std::vector<double> indVec;
    for (int i = 0; i < time.size(); ++i) { indVec.push_back(time[i]); }
    TimeSeriesTable exoGuess(indVec);
    SimTK::Vector zeros(time.size(), 0.0);
    exoGuess.appendColumn("/tau_ankle_angle_l", zeros);
    exoGuess.appendColumn("/tau_ankle_angle_r", zeros);
    guess.insertControlsTrajectory(exoGuess);
    solver.setGuess(guess);

    study.print("sandboxMocoTrack_exoskeleton_markers_MocoStudy.omoco");

    MocoSolution solution = study.solve().unseal();
    solution.write("sandboxMocoTrack_solution_exoskeleton_markers.sto");
    study.visualize(solution);

    return solution;
}

MocoSolution runKneeReactionMinimizationProblem(bool removeMuscles, 
        const std::string& trackedIterateFile) {

    Model model = createModel(removeMuscles);
    if (!removeMuscles) {
        DeGrooteFregly2016Muscle::replaceMuscles(model);
        for (auto& musc : model.updComponentList<DeGrooteFregly2016Muscle>()) {
            musc.set_ignore_passive_fiber_force(true);
        }
    }

    MocoTrack track;
    ModelProcessor modelProcessor = ModelProcessor(model) |
                                    ModOpAddExternalLoads("grf_walk.xml");
    track.setModel(modelProcessor);
    TableProcessor tableProcessor = TableProcessor(trackedIterateFile) |
                                    TabOpLowPassFilter(6);
    track.set_states_reference(tableProcessor);

    // Coordinate tracking weighting map.
    std::map<std::string, double> coordWeightMap;
    // Light tracking of pelvis coordinates (added by us)
    coordWeightMap.insert(std::pair<std::string, double>("pelvis_tx", 1));
    coordWeightMap.insert(std::pair<std::string, double>("pelvis_tz", 1));
    coordWeightMap.insert(std::pair<std::string, double>("pelvis_list", 1));
    coordWeightMap.insert(std::pair<std::string, double>("pelvis_rotation", 1));
    // No tracking of any lower body or lumbar coordinates (Fregly et al. 2007)
    coordWeightMap.insert(std::pair<std::string, double>("lumbar_bending", 0));
    coordWeightMap.insert(std::pair<std::string, double>("lumbar_extension", 0));
    coordWeightMap.insert(std::pair<std::string, double>("lumbar_rotation", 0));
    coordWeightMap.insert(std::pair<std::string, double>("hip_adduction_l", 0));
    coordWeightMap.insert(std::pair<std::string, double>("hip_adduction_r", 0));
    coordWeightMap.insert(std::pair<std::string, double>("hip_flexion_l", 0));
    coordWeightMap.insert(std::pair<std::string, double>("hip_flexion_r", 0));
    coordWeightMap.insert(std::pair<std::string, double>("hip_rotation_l", 0));
    coordWeightMap.insert(std::pair<std::string, double>("hip_rotation_r", 0));
    coordWeightMap.insert(std::pair<std::string, double>("knee_angle_l", 0));
    coordWeightMap.insert(std::pair<std::string, double>("knee_angle_r", 0));
    coordWeightMap.insert(std::pair<std::string, double>("ankle_angle_l", 0));
    coordWeightMap.insert(std::pair<std::string, double>("ankle_angle_r", 0));
    coordWeightMap.insert(std::pair<std::string, double>("subtalar_angle_r", 0));
    coordWeightMap.insert(std::pair<std::string, double>("subtalar_angle_l", 0));

    // Set coordinate tracking weights.
    MocoTrajectory trackedIterate(trackedIterateFile);
    MocoWeightSet coordinateWeights;
    for (const auto& statePath : trackedIterate.getStateNames()) {
        for (auto elt : coordWeightMap) {
            std::string name = elt.first;
            double weight = elt.second;
            if (statePath.find(name) != std::string::npos) {
                coordinateWeights.cloneAndAppend({statePath, weight});
            } else {
                coordinateWeights.cloneAndAppend({statePath, 1000});
            }
        }
    }
    track.set_states_weight_set(coordinateWeights);
    // Keep the low weighted control minimization term to help smooth controls.
    track.set_control_effort_weight(0.001);
    track.set_guess_file(trackedIterateFile);
    track.set_initial_time(0.811);
    track.set_final_time(1.649);

    MocoStudy study = track.initialize();
    auto& problem = study.updProblem();

    // Control tracking cost.
    // ----------------------
    // Control weighting map.
    std::map<std::string, double> controlWeightMap;
    // w6 from Fregly et al. 2007
    controlWeightMap.insert(std::pair<std::string, double>("pelvis_tx", 10));
    controlWeightMap.insert(std::pair<std::string, double>("pelvis_ty", 10));
    controlWeightMap.insert(std::pair<std::string, double>("pelvis_tz", 10));
    controlWeightMap.insert(std::pair<std::string, double>("pelvis_list", 10));
    controlWeightMap.insert(std::pair<std::string, double>("pelvis_tilt", 10));
    controlWeightMap.insert(std::pair<std::string, double>("pelvis_rotation", 10));
    // w2 from Fregly et al. 2007
    controlWeightMap.insert(std::pair<std::string, double>("hip_adduction_l", 0.5));
    controlWeightMap.insert(std::pair<std::string, double>("hip_adduction_r", 0.5));
    controlWeightMap.insert(std::pair<std::string, double>("hip_flexion_l", 0.5));
    controlWeightMap.insert(std::pair<std::string, double>("hip_flexion_r", 0.5));
    controlWeightMap.insert(std::pair<std::string, double>("hip_rotation_l", 0.5));
    controlWeightMap.insert(std::pair<std::string, double>("hip_rotation_r", 0.5));
    controlWeightMap.insert(std::pair<std::string, double>("knee_angle_l", 0.5));
    controlWeightMap.insert(std::pair<std::string, double>("knee_angle_r", 0.5));
    controlWeightMap.insert(std::pair<std::string, double>("ankle_angle_l", 0.5));
    controlWeightMap.insert(std::pair<std::string, double>("ankle_angle_r", 0.5));
    controlWeightMap.insert(std::pair<std::string, double>("subtalar_angle_r", 0.5));
    controlWeightMap.insert(std::pair<std::string, double>("subtalar_angle_l", 0.5));

    // Construct controls reference and weight set.
    auto time = trackedIterate.getTime();
    std::vector<double> timeVec;
    for (int i = 0; i < time.size(); ++i) {
        timeVec.push_back(time[i]);
    }
    TimeSeriesTable controlsRef(timeVec);
    MocoWeightSet controlTrackingWeights;
    for (auto controlPath : trackedIterate.getControlNames()) {
        auto oldControlPath = controlPath;
        auto newControlPath = controlPath.replace(0, 1, "");
        for (auto elt : controlWeightMap) {
            std::string name = elt.first;
            double weight = elt.second;
            if (controlPath.find(name) != std::string::npos) {
                controlTrackingWeights.cloneAndAppend({newControlPath, weight});
                controlsRef.appendColumn(newControlPath,
                    trackedIterate.getControl(oldControlPath));
            } 
        }
    }

    auto* controlTracking =
            problem.addGoal<MocoControlTrackingCost>("control_tracking", 1);
    controlTracking->setReference(controlsRef);
    controlTracking->setWeightSet(controlTrackingWeights);

    // Truck and foot global position tracking costs.
    // ----------------------------------------------
    auto statesRef = trackedIterate.exportToStatesTable();

    // Foot orientation tracking cost.
    // w4 from Fregly et al. 2007
    auto* footOrientationTracking =
            problem.addGoal<MocoOrientationTrackingCost>(
                    "foot_orientation_tracking", 10);
    footOrientationTracking->setStatesReference(statesRef);
    footOrientationTracking->setFramePaths({"/bodyset/calcn_r",
                                            "/bodyset/calcn_l"});
    // Foot translation tracking cost.
    // w4 from Fregly et al. 2007
    auto* footTranslationTracking =
            problem.addGoal<MocoTranslationTrackingCost>(
                    "foot_translation_tracking", 10);
    footTranslationTracking->setStatesReference(statesRef);
    footTranslationTracking->setFramePaths({"/bodyset/calcn_r",
                                            "/bodyset/calcn_l"});
    // Torso orientation tracking cost.
    // w5 from Fregly et al. 2007
    auto* torsoOrientationTracking =
            problem.addGoal<MocoOrientationTrackingCost>(
                    "torso_orientation_tracking", 10);
    torsoOrientationTracking->setStatesReference(statesRef);
    torsoOrientationTracking->setFramePaths({"/bodyset/torso"});

    // Knee adduction cost.
    // --------------------
    // w1 from Fregly et al. 2007
    auto* kneeAdductionCost_l =
            problem.addGoal<MocoJointReactionCost>("knee_adduction_cost_l", 10);
    kneeAdductionCost_l->setJointPath("/jointset/walker_knee_l");
    kneeAdductionCost_l->setExpressedInFramePath("/bodyset/tibia_l");
    kneeAdductionCost_l->setReactionMeasures({"moment_x"});
    auto* kneeAdductionCost_r =
            problem.addGoal<MocoJointReactionCost>("knee_adduction_cost_r", 10);
    kneeAdductionCost_r->setJointPath("/jointset/walker_knee_r");
    kneeAdductionCost_r->setExpressedInFramePath("/bodyset/tibia_r");
    kneeAdductionCost_r->setReactionMeasures({"moment_x"});

    // Configure solver.
    // -----------------
    auto& solver = study.updSolver<MocoCasADiSolver>();
    solver.set_optim_constraint_tolerance(1e-2);
    solver.set_optim_convergence_tolerance(1e-2);

    // Solve!
    // ------
    MocoSolution solution = study.solve().unseal();
    std::string baseline;
    std::string filename;
    if (removeMuscles) {
        baseline = "sandboxMocoTrack_solution_baseline.sto";
        filename = "sandboxMocoTrack_solution_minimize_knee_adduction.sto";
    } else {
        baseline = "sandboxMocoTrack_solution_baseline_muscles.sto";
        filename = 
            "sandboxMocoTrack_solution_minimize_knee_adduction_muscles.sto";
    }
    solution.write(filename);
    study.visualize(solution);

    // Compute reaction loads.
    // -----------------------
    TimeSeriesTable_<SimTK::SpatialVec> reactionTableMTG =
            analyze<SimTK::SpatialVec>(model, solution,
                {"/jointset/walker_knee_l/reaction_on_child",
                 "/jointset/walker_knee_r/reaction_on_child"});

    transformReactionToBodyFrame(study, solution, reactionTableMTG);
    STOFileAdapter::write(reactionTableMTG.flatten(),
        filename.replace(filename.end()-4, filename.end(), "_reactions.sto"));

    // Get "smoothed" solution.
    // ------------------------
    smoothSolutionControls(model, filename, filename);

    // Compute and print transform errors.
    // -----------------------------------
    auto transformErrors = computeTransformErrors(model, baseline, 
            filename.replace(filename.end()-4, filename.end(), "_smoothed.sto"),
            {"/bodyset/calcn_r", "/bodyset/calcn_l", "/bodyset/torso"});
    STOFileAdapter::write(transformErrors, 
        filename.replace(filename.end() - 4, filename.end(), 
            "_transform_errors.sto"));

    for (int i = 0; i < transformErrors.getNumColumns(); ++i) {
        std::cout << transformErrors.getColumnLabel(i) << ": ";
        std::cout << transformErrors.getDependentColumnAtIndex(i).normRMS();
        std::cout << std::endl;
    }

    return solution;
}

MocoSolution runExoskeletonProblem(const std::string& trackedIterateFile, 
        double stateTrackingWeight, double controlEffortWeight) {

    // Configure model.
    // ----------------
    Model model = createModel(false, true);
    DeGrooteFregly2016Muscle::replaceMuscles(model);
    for (auto& musc : model.updComponentList<DeGrooteFregly2016Muscle>()) {
        musc.set_ignore_passive_fiber_force(true);
        musc.set_active_force_width_scale(1.5);
    }

    // Configure tracking tool.
    // ------------------------
    MocoTrack track;
    ModelProcessor modelProcessor = ModelProcessor(model) |
                                    ModOpAddExternalLoads("grf_walk.xml");
    track.setModel(model);
    TableProcessor tableProcessor = 
            TableProcessor("coordinates_rra_adjusted.sto") |
            TabOpLowPassFilter(6);
    track.set_states_reference(tableProcessor);
    track.set_track_reference_position_derivatives(true);    
    track.set_guess_file("sandboxMocoTrack_solution_exoskeleton_muscles.sto");
    track.set_initial_time(0.811);
    track.set_final_time(1.649);

    // Cost weights.
    // -------------
    track.set_states_global_tracking_weight(stateTrackingWeight);
    track.set_control_effort_weight(controlEffortWeight);

    // Initialize MocoStudy and grab problem.
    // --------------------------------------
    MocoStudy study = track.initialize();
    auto& problem = moco.updProblem();
    // Don't penalize exoskeleton.
    auto& effort = dynamic_cast<MocoControlGoal&>(
            study.updProblem().updGoal("control_effort"));
    double coordActWeight = 10000;
    effort.setWeight("/tau_ankle_angle_r", 0.001*controlEffortWeight);
    effort.setWeight("/tau_ankle_angle_l", 0.001*controlEffortWeight);
    effort.setWeight("/tau_subtalar_angle_r", 0.001*controlEffortWeight);
    effort.setWeight("/tau_subtalar_angle_l", 0.001*controlEffortWeight);
    effort.setWeight("/tau_lumbar_bending", 0.001*controlEffortWeight);
    effort.setWeight("/tau_lumbar_extension", 0.001*controlEffortWeight);
    effort.setWeight("/tau_lumbar_rotation", 0.001*controlEffortWeight);
    effort.setWeight("/tau_pelvis_tx", coordActWeight*controlEffortWeight);
    effort.setWeight("/tau_pelvis_ty", 0);
    effort.setWeight("/tau_pelvis_tz", coordActWeight*controlEffortWeight);
    effort.setWeight("/tau_pelvis_list", coordActWeight*controlEffortWeight);
    effort.setWeight("/tau_pelvis_tilt", coordActWeight*controlEffortWeight);
    effort.setWeight("/tau_pelvis_rotation", coordActWeight*controlEffortWeight);
    effort.setWeight("/tau_hip_adduction_l", coordActWeight*controlEffortWeight);
    effort.setWeight("/tau_hip_adduction_r", coordActWeight*controlEffortWeight);
    effort.setWeight("/tau_hip_flexion_l", coordActWeight*controlEffortWeight);
    effort.setWeight("/tau_hip_flexion_r", coordActWeight*controlEffortWeight);
    effort.setWeight("/tau_hip_rotation_l", coordActWeight*controlEffortWeight);
    effort.setWeight("/tau_hip_rotation_r", coordActWeight*controlEffortWeight);
    effort.setWeight("/tau_knee_angle_l", coordActWeight*controlEffortWeight);
    effort.setWeight("/tau_knee_angle_r", coordActWeight*controlEffortWeight);

    // Truck and foot global position tracking costs.
    // ----------------------------------------------
    MocoTrajectory trackedIterate(trackedIterateFile);
    auto statesRef = trackedIterate.exportToStatesTable();

    // Foot orientation tracking cost.
    // w4 from Fregly et al. 2007
    auto* footOrientationTracking =
            problem.addGoal<MocoOrientationTrackingCost>(
                    "foot_orientation_tracking", 1e6);
    footOrientationTracking->setStatesReference(statesRef);
    footOrientationTracking->setFramePaths({"/bodyset/calcn_r",
        "/bodyset/calcn_l"});
    // Foot translation tracking cost.
    // w4 from Fregly et al. 2007
    auto* footTranslationTracking =
            problem.addGoal<MocoTranslationTrackingCost>(
                    "foot_translation_tracking", 1e6);
    footTranslationTracking->setStatesReference(statesRef);
    footTranslationTracking->setFramePaths({"/bodyset/calcn_r",
        "/bodyset/calcn_l"});
    // Torso orientation tracking cost.
    // w5 from Fregly et al. 2007
    auto* torsoOrientationTracking =
            problem.addGoal<MocoOrientationTrackingCost>(
                    "torso_orientation_tracking", 1e6);
    torsoOrientationTracking->setStatesReference(statesRef);
    torsoOrientationTracking->setFramePaths({"/bodyset/torso"});

    // Configure solver.
    // -----------------
    auto& solver = study.updSolver<MocoCasADiSolver>();
    solver.set_optim_constraint_tolerance(1e-2);
    solver.set_optim_convergence_tolerance(1e-2);

    // Save to setup file.
    study.print("sandboxMocoTrack_exoskeleton_muscles.omoco");

    // Solve!
    // ------
    MocoSolution solution = study.solve().unseal();
    solution.write("sandboxMocoTrack_solution_exoskeleton_muscles.sto");
    study.visualize(solution);

    return solution;
}

int main() {

    // Baseline tracking problem w/o muscles.
    // --------------------------------------
    MocoSolution baseline = runBaselineProblem(true, controlWeight);

    // Baseline tracking problem w/ muscles.
    // -------------------------------------
    //MocoSolution baselineWithMuscles = runBaselineProblem(false, 0.1,
    //    "sandboxMocoTrack_solution_baseline_muscles.sto");

    // Knee adduction minimization w/o muscles.
    // ----------------------------------------
    //MocoSolution kneeAdducMin = runKneeReactionMinimizationProblem(true, 
    //    "sandboxMocoTrack_solution_baseline.sto");

    // Minimize effort with exoskeleton device w/ muscles.
    // ---------------------------------------------------
    //runExoskeletonProblem("sandboxMocoTrack_solution_baseline_muscles.sto",
       // 0.1, controlWeight);

    MocoSolution exoskeletonSolution = runExoskeletonProblemMarkers(
            1.0, "sandboxMocoTrack_solution_baseline_muscles_keep.sto");
    

    return EXIT_SUCCESS;
}
