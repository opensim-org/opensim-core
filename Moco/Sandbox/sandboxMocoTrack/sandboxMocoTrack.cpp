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
#include <OpenSim/Common/LogManager.h>

#include <OpenSim/OpenSim.h>

namespace OpenSim {


class COPTrackingCost : public MocoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(COPTrackingCost, MocoCost);
public:
    COPTrackingCost() {}
    COPTrackingCost(std::string name, double weight)
        : MocoCost(std::move(name), weight) {}

    void setReference(const TimeSeriesTable& ref) {
        m_ref = ref;
    }
    void setExternalForceNames(const std::vector<std::string>& names) {
        m_external_force_names = names;
    }
protected:
    void initializeOnModelImpl(const Model& model) const override {

        auto colLabels = m_ref.getColumnLabels();
        m_refsplines = GCVSplineSet(m_ref, colLabels);
        std::vector<std::string> suffixes = {"x", "y", "z"};

        for (const auto& extForceName : m_external_force_names) {
            const auto& extForce = 
                    model.getComponent<ExternalForce>(extForceName);
            m_model_ext_forces.emplace_back(&extForce);

            // Find the reference data column labels that match the COP names
            // and save their indices. 
            for (int i = 0; i < colLabels.size(); ++i) {
                for (const auto& suffix : suffixes) {
                    if (colLabels[i] == 
                            (extForce.getPointIdentifier() + suffix)) {
                        m_refindices.push_back(i);
                    }
                }
            }
        }
    }

    void calcIntegralCostImpl(const SimTK::State& state,
            double& integrand) const override {
        const auto& time = state.getTime();
        // Need to realize to velocity to get controls.
        getModel().realizeDynamics(state);
        SimTK::Vector timeVec(1, time);

        for (int iforce = 0; iforce < m_model_ext_forces.size(); ++iforce) {
            const auto& extforce = m_model_ext_forces[iforce];

            int point_x_idx = m_refindices[3*iforce];
            int point_y_idx = m_refindices[3*iforce + 1];
            int point_z_idx = m_refindices[3*iforce + 2];

            SimTK::Vec3 copRef(
                    m_refsplines[point_x_idx].calcValue(timeVec),
                    m_refsplines[point_y_idx].calcValue(timeVec),
                    m_refsplines[point_z_idx].calcValue(timeVec));

            SimTK::Vec3 copModel = extforce->getPointAtTime(time);
            
            // Convert points to body frame.
            // TODO: this assumes that the data is in the same frame as the
            // COP in the model actuator.
            if (extforce->getPointExpressedInBodyName() == "ground") {

                const auto& bodyName = extforce->getAppliedToBodyName();
                const auto& body = 
                        getModel().getComponent<Body>("/bodyset/" + bodyName);

                copModel = getModel().getGround().
                    findStationLocationInAnotherFrame(state, copModel,
                        body);
            }

            integrand += (copModel - copRef).normSqr() / copRef.norm();

        }
    }

private:
    TimeSeriesTable m_ref;
    mutable std::vector<int> m_refindices;
    mutable GCVSplineSet m_refsplines;
    std::vector<std::string> m_external_force_names;
    mutable std::vector<SimTK::ReferencePtr<const ExternalForce>> 
    m_model_ext_forces;

};

} // namespace OpenSim

using namespace OpenSim;

void addCoordinateActuator(Model& model, std::string coordName, 
        double optimalForce) {

    auto& coordSet = model.updCoordinateSet();

    auto* actu = new CoordinateActuator();
    actu->setName("tau_" + coordName);
    actu->setCoordinate(&coordSet.get(coordName));
    actu->setOptimalForce(optimalForce);
    actu->setMinControl(-1);
    actu->setMaxControl(1);
    model.addComponent(actu);
}

void transformReactionToBodyFrame(const MocoStudy& moco, 
        const MocoIterate& iterate, 
        TimeSeriesTable_<SimTK::SpatialVec>& reactionTable) {
    auto model = moco.getProblem().createRep().getModelBase();
    model.initSystem();
    const auto& ground = model.getGround();
    auto statesTraj = iterate.exportToStatesTrajectory(moco.getProblem());
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

void transformExternalForceToBodyFrame(const MocoStudy& moco,
        const MocoIterate& iterate,
        const std::string& extLoadsFile) {

    auto model = moco.getProblem().createRep().getModelBase();
    model.initSystem();

    const auto& ground = model.getGround();

    auto statesTraj = iterate.exportToStatesTrajectory(moco.getProblem());
    std::vector<double> time;
    for (const auto& state : statesTraj) {
        time.push_back(state.getTime());
    }
    TimeSeriesTableVec3 table(time);
    for (const auto& extForce : model.getComponentList<ExternalForce>()) {
        const auto& bodyName = extForce.getAppliedToBodyName();
        const auto& body = model.getComponent<Body>("/bodyset/" + bodyName);

        SimTK::Vector_<SimTK::Vec3> moment(statesTraj.getSize());
        SimTK::Vector_<SimTK::Vec3> force(statesTraj.getSize());
        SimTK::Vector_<SimTK::Vec3> point(statesTraj.getSize());
        for (int istate = 0; istate < time.size(); ++istate) {
            const auto& state = statesTraj.get(istate);
            model.realizeAcceleration(state);

            auto time = state.getTime();
            SimTK::Vec3 momentInGround = extForce.getTorqueAtTime(time);
            SimTK::Vec3 forceInGround = extForce.getForceAtTime(time);
            SimTK::Vec3 pointInGround = extForce.getPointAtTime(time);

            moment[istate] = ground.expressVectorInAnotherFrame(state,
                momentInGround, body);
            force[istate] = ground.expressVectorInAnotherFrame(state,
                forceInGround, body);
            point[istate] = ground.expressVectorInAnotherFrame(state,
                pointInGround, body);
        }

        table.appendColumn(extForce.getTorqueIdentifier(), moment);
        table.appendColumn(extForce.getForceIdentifier(), force);
        table.appendColumn(extForce.getPointIdentifier(), point);
    }

    TimeSeriesTable tableFlat = table.flatten({"x", "y", "z"});
    STOFileAdapter::write(tableFlat, "forces_transformed_to_body.sto");
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

    for (int i = 0; i < time.size(); ++i) {
        SimTK::RowVector error(4*compPaths.size(), 0.0);
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

Model createModel(bool keepMuscles = false) {

    Model model("subject_walk_rra_adjusted.osim");
    if (!keepMuscles) model.updForceSet().clearAndDestroy();
    model.initSystem();

    // weld joints w/ locked coordinates
    //ModelFactory::replaceJointWithWeldJoint(model, "subtalar_l");
    //ModelFactory::replaceJointWithWeldJoint(model, "subtalar_r");
    ModelFactory::replaceJointWithWeldJoint(model, "mtp_l");
    ModelFactory::replaceJointWithWeldJoint(model, "mtp_r");
    ModelFactory::replaceJointWithWeldJoint(model, "radius_hand_l");
    ModelFactory::replaceJointWithWeldJoint(model, "radius_hand_r");
    // lower body
    addCoordinateActuator(model, "pelvis_tilt", 10);
    addCoordinateActuator(model, "pelvis_list", 10);
    addCoordinateActuator(model, "pelvis_rotation", 10);
    addCoordinateActuator(model, "pelvis_tx", 10);
    addCoordinateActuator(model, "pelvis_ty", 10);
    addCoordinateActuator(model, "pelvis_tz", 10);
    if (!keepMuscles) {
        addCoordinateActuator(model, "hip_adduction_l", 100);
        addCoordinateActuator(model, "hip_adduction_r", 100);
        addCoordinateActuator(model, "hip_flexion_l", 100);
        addCoordinateActuator(model, "hip_flexion_r", 100);
        addCoordinateActuator(model, "hip_rotation_l", 20);
        addCoordinateActuator(model, "hip_rotation_r", 20);
        addCoordinateActuator(model, "knee_angle_l", 100);
        addCoordinateActuator(model, "knee_angle_r", 100);
        addCoordinateActuator(model, "ankle_angle_l", 250);
        addCoordinateActuator(model, "ankle_angle_r", 250);
        addCoordinateActuator(model, "subtalar_angle_r", 100);
        addCoordinateActuator(model, "subtalar_angle_l", 100);
    }
    // upper body
    addCoordinateActuator(model, "elbow_flex_l", 5);
    addCoordinateActuator(model, "elbow_flex_r", 5);
    addCoordinateActuator(model, "pro_sup_l", 0.1);
    addCoordinateActuator(model, "pro_sup_r", 0.1);
    addCoordinateActuator(model, "arm_add_l", 5);
    addCoordinateActuator(model, "arm_add_r", 5);
    addCoordinateActuator(model, "arm_rot_l", 5);
    addCoordinateActuator(model, "arm_rot_r", 5);
    addCoordinateActuator(model, "arm_flex_l", 10);
    addCoordinateActuator(model, "arm_flex_r", 10);
    addCoordinateActuator(model, "lumbar_bending", 100);
    addCoordinateActuator(model, "lumbar_extension", 100);
    addCoordinateActuator(model, "lumbar_rotation", 100);

    if (keepMuscles) {
        model.print("subject_walk_rra_adjusted_updated_muscles.osim");
    } else {
        model.print("subject_walk_rra_adjusted_updated.osim");
    }

    return model;
}

void smoothSolutionControls(std::string statesFile, 
        const std::string& guessFile) {

    std::cout.rdbuf(LogManager::cout.rdbuf());
    std::cerr.rdbuf(LogManager::cerr.rdbuf());

    Model model = createModel();

    MocoTrack track;
    track.setName("smoothed");
    track.setModel(model);
    track.set_states_tracking_file(statesFile);
    track.set_lowpass_cutoff_frequency_for_kinematics(6);
    track.set_external_loads_file("grf_walk.xml");
    track.set_guess_type("from_file");
    track.set_guess_file(guessFile);
    track.set_initial_time(0.812);
    track.set_final_time(1.648);

    track.set_minimize_controls(0.05);

    MocoStudy moco = track.initialize();

    auto& solver = moco.updSolver<MocoCasADiSolver>();
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_convergence_tolerance(1e-4);

    MocoProblem problem = moco.getProblem();

    MocoSolution solution = moco.solve().unseal();
    solution.write(statesFile.replace(statesFile.end()-4, statesFile.end(),
        "_smoothed.sto"));
    moco.visualize(solution);

    TimeSeriesTable_<SimTK::SpatialVec> reactionTable =
        moco.analyze<SimTK::SpatialVec>(solution,
        {"/jointset/walker_knee_l/reaction_on_child",
            "/jointset/walker_knee_r/reaction_on_child"});

    transformReactionToBodyFrame(moco, solution, reactionTable);
    TimeSeriesTable reactionTableFlat = reactionTable.flatten();
    STOFileAdapter::write(reactionTableFlat, 
        statesFile.replace(statesFile.end() - 4, statesFile.end(),
            "_reactions.sto"));
}

//void solveInverseProblem(std::string kinematicsFile, 
//        bool minimizeReactions = false) {
//
//    auto modelWithMuscles = createModel(true);
//    DeGrooteFregly2016Muscle::replaceMuscles(modelWithMuscles);
//    for (auto& musc : 
//            modelWithMuscles.updComponentList<DeGrooteFregly2016Muscle>()) {
//        musc.set_ignore_passive_fiber_force(true);
//        //musc.set_max_isometric_force(1000*musc.get_max_isometric_force());
//    }
//
//    MocoInverse inverse;
//    inverse.setModel(modelWithMuscles);
//    inverse.set_initial_time(0.815);
//    inverse.set_final_time(1.645);
//    inverse.set_kinematics_file(kinematicsFile);
//    inverse.set_lowpass_cutoff_frequency_for_kinematics(6);
//    inverse.set_kinematics_allow_extra_columns(true);
//    inverse.set_external_loads_file("grf_walk.xml");
//    inverse.set_ignore_tendon_compliance(true);
//    inverse.set_mesh_interval(0.05);
//    inverse.set_create_reserve_actuators(1);
//    inverse.set_minimize_sum_squared_states(true);
//    MocoStudy moco = inverse.initialize();
//    if (minimizeReactions) {
//        auto& problem = moco.updProblem();
//        auto* kneeAdductionCost_l =
//            problem.addCost<MocoJointReactionCost>("knee_adduction_cost_l", 0.01);
//        kneeAdductionCost_l->setJointPath("/jointset/walker_knee_l");
//        kneeAdductionCost_l->setExpressedInFramePath("/bodyset/tibia_l");
//        kneeAdductionCost_l->setReactionComponent(0);
//        auto* kneeAdductionCost_r =
//            problem.addCost<MocoJointReactionCost>("knee_adduction_cost_r", 0.01);
//        kneeAdductionCost_r->setJointPath("/jointset/walker_knee_r");
//        kneeAdductionCost_r->setExpressedInFramePath("/bodyset/tibia_r");
//        kneeAdductionCost_r->setReactionComponent(0);
//        auto* kneeVerticalForceCost_l =
//            problem.addCost<MocoJointReactionCost>("knee_vertical_force_cost_l", 0.01);
//        kneeVerticalForceCost_l->setJointPath("/jointset/walker_knee_l");
//        kneeVerticalForceCost_l->setExpressedInFramePath("/bodyset/tibia_l");
//        kneeVerticalForceCost_l->setReactionComponent(4);
//        auto* kneeVerticalForceCost_r =
//            problem.addCost<MocoJointReactionCost>("knee_vertical_force_cost_r", 0.01);
//        kneeVerticalForceCost_r->setJointPath("/jointset/walker_knee_r");
//        kneeVerticalForceCost_r->setExpressedInFramePath("/bodyset/tibia_r");
//        kneeVerticalForceCost_r->setReactionComponent(4);
//    }
//    auto inverseSolution = moco.solve();
//
//    MocoIterate prevSol(kinematicsFile);
//    std::vector<double> time;
//    for (int i = 0; i < prevSol.getTime().size(); ++i) {
//        time.push_back(prevSol.getTime()[i]);
//    }
//
//    inverseSolution.write("temp_inverse_sol.sto");
//    auto inverseSol = STOFileAdapter::read("temp_inverse_sol.sto");
//    auto invInitTime = inverseSol.getIndependentColumn().at(0);
//    auto invFinalTime = inverseSol.getIndependentColumn().at(
//            inverseSol.getNumRows()-1);
//
//    GCVSplineSet inverseSplines(inverseSol);
//    TimeSeriesTable controls(time);
//    TimeSeriesTable states(time);
//    std::vector<std::string> controlLabels;
//    std::vector<std::string> stateLabels;
//    for (int i = 0; i < inverseSol.getNumColumns(); ++i) {
//        std::string label = inverseSol.getColumnLabel(i);
//        auto col = inverseSol.getDependentColumnAtIndex(i);
//        if (label.find("activation") != std::string::npos) {
//            stateLabels.push_back(label);
//            SimTK::Vector state(time.size());
//            for (int t = 0; t < time.size(); ++t) {
//                if (time[t] < invInitTime || time[t] > invFinalTime) {
//                    state[t] = 0.0;
//                } else {
//                    SimTK::Vector timeVec(1, time[t]);
//                    state[t] = inverseSplines.get(label).calcValue(timeVec);
//                }
//            }
//            states.appendColumn(label, state);
//        } else {
//            controlLabels.push_back(label);
//            SimTK::Vector control(time.size());
//            for (int t = 0; t < time.size(); ++t) {
//                if (time[t] < invInitTime || time[t] > invFinalTime) {
//                    control[t] = 0.0;
//                } else {
//                    SimTK::Vector timeVec(1, time[t]);
//                    control[t] = inverseSplines.get(label).calcValue(timeVec);
//                }
//            }
//            controls.appendColumn(label, control);
//        }
//    }
//
//    for (const auto& stateLabel : prevSol.getStateNames()) {
//        stateLabels.push_back(stateLabel);
//        states.appendColumn(stateLabel, prevSol.getState(stateLabel));
//    }
//
//
//    MocoIterate inverseIterate(prevSol.getTime(), stateLabels, controlLabels,
//        prevSol.getMultiplierNames(), {}, states.getMatrix(), 
//        controls.getMatrix(), prevSol.getMultipliersTrajectory(), {});
//
//
//    if (minimizeReactions) {
//        inverseIterate.write(kinematicsFile.replace(
//            kinematicsFile.end() - 4, kinematicsFile.end(),
//            "_inverse_min_reactions.sto"));
//    } else {
//        inverseIterate.write(kinematicsFile.replace(
//            kinematicsFile.end() - 4, kinematicsFile.end(), "_inverse.sto"));
//    }
//
//    TimeSeriesTable_<SimTK::SpatialVec> reactionTable = 
//            moco.analyze<SimTK::SpatialVec>(inverseIterate,
//                {"/jointset/walker_knee_l/reaction_on_child",
//                 "/jointset/walker_knee_r/reaction_on_child"});
//
//    transformReactionToBodyFrame(moco, inverseIterate, reactionTable);
//    TimeSeriesTable reactionTableFlat = reactionTable.flatten();
//
//    STOFileAdapter::write(reactionTableFlat, kinematicsFile.replace(
//        kinematicsFile.end() - 4, kinematicsFile.end(), "_reactions.sto"));
//
//}   

int main() {

    Model model = createModel();

    // Baseline tracking problem.
    // --------------------------
    MocoTrack track;
    track.setName("baseline");
    track.setModel(model);
    track.set_states_tracking_file("coordinates_rra_adjusted.sto");
    //MocoWeightSet stateWeights;
    //MocoSolution prevSol("sandboxMocoTrack_solution.sto");
    //for (const auto& stateName : prevSol.getStateNames()) {
    //    if (stateName.find("pelvis") != std::string::npos) {
    //        stateWeights.cloneAndAppend({stateName, 1});
    //    } else if (stateName.find("lumbar") != std::string::npos) {
    //        // From Fregly et al. 2007: unlock all back rotations.
    //        stateWeights.cloneAndAppend({stateName, 1});
    //    } else if (stateName.find("hip") != std::string::npos ||
    //               stateName.find("knee") != std::string::npos ||
    //               stateName.find("ankle") != std::string::npos) {
    //        // From Fregly et al. 2007: unlock all hip, knee, and ankle 
    //        // rotations.
    //        stateWeights.cloneAndAppend({stateName, 1});
    //    } else {
    //        stateWeights.cloneAndAppend({stateName, 1});
    //    }
    //}
    //track.set_state_weights(stateWeights);
    track.set_track_state_reference_derivatives(true);
    //track.set_markers_tracking_file("motion_capture_walk.trc");
    //track.set_ik_setup_file("ik_setup_walk_feet_only.xml");
    track.set_external_loads_file("grf_walk.xml");
    track.set_initial_time(0.81);
    track.set_final_time(1.65);

    track.set_minimize_controls(0.05);
    MocoWeightSet controlWeights;
    controlWeights.cloneAndAppend({"tau_subtalar_angle_r", 10});
    controlWeights.cloneAndAppend({"tau_subtalar_angle_l", 10});
    track.set_control_weights(controlWeights);

    MocoStudy moco = track.initialize();
    auto& solver = moco.updSolver<MocoCasADiSolver>();
    solver.set_optim_constraint_tolerance(1e-3);
    solver.set_optim_convergence_tolerance(1e-3);

    MocoProblem problem = moco.getProblem();
    //MocoSolution solution = moco.solve().unseal();
    //solution.write("sandboxMocoTrack_solution_baseline.sto");
    //moco.visualize(solution);

    //TimeSeriesTable_<SimTK::SpatialVec> reactionTable =
    //        moco.analyze<SimTK::SpatialVec>(solution,
    //            {"/jointset/walker_knee_l/reaction_on_child",
    //             "/jointset/walker_knee_r/reaction_on_child"});

    //transformReactionToBodyFrame(moco, solution, reactionTable);
    //TimeSeriesTable reactionTableFlat = reactionTable.flatten();    
    //STOFileAdapter::write(reactionTableFlat, 
    //    "sandboxMocoTrack_solution_baseline_reactions.sto");
    MocoSolution solution("sandboxMocoTrack_solution_baseline.sto");

    // Medial thrust gait.
    // -------------------
    MocoTrack trackMTG;
    trackMTG.setModel(model);
    trackMTG.set_states_tracking_file("sandboxMocoTrack_solution_baseline.sto");
    // Track 
    MocoWeightSet coordinateWeights;
    for (const auto& stateName : solution.getStateNames()) {
        if (stateName.find("pelvis") != std::string::npos) {
            if (stateName.find("pelvis_tx") != std::string::npos) {
                coordinateWeights.cloneAndAppend({stateName, 1000});
            } else if (stateName.find("pelvis_tz") != std::string::npos) {
                coordinateWeights.cloneAndAppend({stateName, 10000});
            } else if (stateName.find("pelvis_list") != std::string::npos) {
                coordinateWeights.cloneAndAppend({stateName, 100});
            } else if (stateName.find("pelvis_rotation") != std::string::npos) {
                coordinateWeights.cloneAndAppend({stateName, 5});
            } else {
                // From Fregly et al. 2007: unlock superior/inferior translation
                // and all pelvis rotations.
                coordinateWeights.cloneAndAppend({stateName, 0.0});
            }
            //coordinateWeights.cloneAndAppend({stateName, 1000});

        } else if (stateName.find("lumbar") != std::string::npos) {
            // From Fregly et al. 2007: unlock all back rotations.
            coordinateWeights.cloneAndAppend({stateName, 0.0});
        } else if (stateName.find("hip") != std::string::npos ||
                   stateName.find("knee") != std::string::npos ||
                   stateName.find("ankle") != std::string::npos ||
                   stateName.find("subtalar") != std::string::npos) {
            // From Fregly et al. 2007: unlock all hip, knee, and ankle 
            // rotations.
            coordinateWeights.cloneAndAppend({stateName, 0.0});
        } else {
            coordinateWeights.cloneAndAppend({stateName, 1000});
        }
    }
    trackMTG.set_state_weights(coordinateWeights);
    trackMTG.set_external_loads_file("grf_walk.xml");

    // Keep the low weighted control minimization term to help smooth controls.
    //trackMTG.set_minimize_controls(0.001);
    trackMTG.set_guess_type("from_file");
    trackMTG.set_guess_file("sandboxMocoTrack_solution_baseline.sto");
    trackMTG.set_initial_time(0.811);
    trackMTG.set_final_time(1.649);
    MocoStudy mocoMTG = trackMTG.initialize();
    auto& problemMTG = mocoMTG.updProblem();

    // COP tracking (in the body frame).
    // w3 from Fregly et al. 2007
    //auto* copTracking = problemMTG.addCost<COPTrackingCost>("cop_tracking", 
    //        10);
    //copTracking->setReference(TimeSeriesTable("forces_new_labels.mot"));
    //copTracking->setFreePointBodyActuatorNames({"Left_GRF", "Right_GRF"});

    // Control tracking cost.
    // Construct controls reference.
    auto time = solution.getTime();
    std::vector<double> timeVec;
    for (int i = 0; i < time.size(); ++i) {
        timeVec.push_back(time[i]);
    }
    TimeSeriesTable controlsRef(timeVec);
    // Control weights.
    MocoWeightSet controlTrackingWeights;
    for (auto controlName : solution.getControlNames()) {
        auto oldControlName = controlName;
        auto newControlName = controlName.replace(0, 1, "");

        if (controlName.find("pelvis") != std::string::npos) {
            // w6 from Fregly et al. 2007
            controlTrackingWeights.cloneAndAppend({newControlName, 1000});
            controlsRef.appendColumn(newControlName,
                solution.getControl(oldControlName));
        } else if (controlName.find("hip") != std::string::npos ||
                   controlName.find("knee") != std::string::npos ||
                   controlName.find("ankle") != std::string::npos || 
                   controlName.find("subtalar") != std::string::npos) {
            // w2 from Fregly et al. 2007
            controlTrackingWeights.cloneAndAppend({newControlName, 0.001});
            controlsRef.appendColumn(newControlName,
                solution.getControl(oldControlName));
        }
    }
    auto* controlTracking =
            problemMTG.addCost<MocoControlTrackingCost>("control_tracking", 1);
    controlTracking->setReference(controlsRef);
    controlTracking->setWeightSet(controlTrackingWeights);

    // Feet transform tracking cost.
    auto statesTraj = solution.exportToStatesTrajectory(problem);
    // w4 from Fregly et al. 2007
    auto* footTransformTrackingRotation =
            problemMTG.addCost<TransformTrackingCost>("foot_tracking_rotation", 2000);
    footTransformTrackingRotation->setStatesTrajectory(statesTraj);
    footTransformTrackingRotation->setComponentPaths({"/bodyset/calcn_r",
            "/bodyset/calcn_l"});
    footTransformTrackingRotation->setTrackedComponents("rotation");

    auto* footTransformTrackingPosition =
        problemMTG.addCost<TransformTrackingCost>("foot_tracking", 2000);
    footTransformTrackingPosition->setStatesTrajectory(statesTraj);
    footTransformTrackingPosition->setComponentPaths({"/bodyset/calcn_r",
        "/bodyset/calcn_l"});
    footTransformTrackingPosition->setTrackedComponents("position");

    // Torso transform tracking cost.
    // w5 from Fregly et al. 2007
    auto* torsoTransformTracking =
        problemMTG.addCost<TransformTrackingCost>("torso_tracking", 2000);
    torsoTransformTracking->setStatesTrajectory(statesTraj);
    torsoTransformTracking->setComponentPaths({"/bodyset/torso"});
    torsoTransformTracking->setTrackedComponents("rotation");

    // Knee adduction cost.
    // w1 from Fregly et al. 2007
    auto* kneeAdductionCost_l = 
        problemMTG.addCost<MocoJointReactionCost>("knee_adduction_cost_l", 100);
    kneeAdductionCost_l->setJointPath("/jointset/walker_knee_l");
    kneeAdductionCost_l->setExpressedInFramePath("/bodyset/tibia_l");
    kneeAdductionCost_l->setReactionComponent(0);
    auto* kneeAdductionCost_r =
        problemMTG.addCost<MocoJointReactionCost>("knee_adduction_cost_r", 10);
    kneeAdductionCost_r->setJointPath("/jointset/walker_knee_r");
    kneeAdductionCost_r->setExpressedInFramePath("/bodyset/tibia_r");
    kneeAdductionCost_r->setReactionComponent(0);

    auto& solverMTG = mocoMTG.updSolver<MocoCasADiSolver>();
    solverMTG.set_optim_constraint_tolerance(1e-2);
    solverMTG.set_optim_convergence_tolerance(1e-2);

    //MocoSolution solutionMTG = mocoMTG.solve().unseal();
    //solutionMTG.write("sandboxMocoTrack_solution_MTG.sto");
    //mocoMTG.visualize(solutionMTG);
    ////MocoSolution solutionMTG("sandboxMocoTrack_solution_MTG.sto");

    //TimeSeriesTable_<SimTK::SpatialVec> reactionTableMTG = 
    //        mocoMTG.analyze<SimTK::SpatialVec>(solutionMTG,
    //            {"/jointset/walker_knee_l/reaction_on_child",
    //             "/jointset/walker_knee_r/reaction_on_child"});

    //transformReactionToBodyFrame(mocoMTG, solutionMTG, reactionTableMTG);
    //TimeSeriesTable reactionTableFlatMTG = reactionTableMTG.flatten();

    //STOFileAdapter::write(reactionTableFlatMTG, 
    //        "sandboxMocoTrack_solution_MTG_reactions.sto");

    //smoothSolutionControls("sandboxMocoTrack_solution_MTG.sto",
    //        "sandboxMocoTrack_solution_MTG.sto");

    //auto transformErrors = computeTransformErrors(model, 
    //    "sandboxMocoTrack_solution_baseline.sto",
    //    "sandboxMocoTrack_solution_MTG_smoothed.sto", 
    //    {"/bodyset/calcn_r", "/bodyset/calcn_l", "/bodyset/torso"});

    //STOFileAdapter::write(transformErrors, 
    //    "sandboxMocoTrack_transform_errors.sto");

    //for (int i = 0; i < transformErrors.getNumColumns(); ++i) {
    //    std::cout << transformErrors.getColumnLabel(i) << ": ";
    //    std::cout << transformErrors.getDependentColumnAtIndex(i).normRMS(); 
    //    std::cout << std::endl;
    //}

    //std::cout << std::endl;
    
    // Inverse problems.
    solveInverseProblem("sandboxMocoTrack_solution_baseline.sto");
    solveInverseProblem("sandboxMocoTrack_solution_baseline.sto", true);
    solveInverseProblem("sandboxMocoTrack_solution_MTG_smoothed.sto");
    solveInverseProblem("sandboxMocoTrack_solution_MTG_smoothed.sto", true);


    std::cout << std::endl;

    return EXIT_SUCCESS;
}
