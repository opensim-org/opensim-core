/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleCoordinateTracking.cpp                                *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Antoine Falisse                                                 *
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

#include <Moco/osimMoco.h>

using namespace OpenSim;
// This class defines a MocoCost that encourages symmetry of the walking cycle.
// The cost minimizes the squared difference between the initial and final
// states of controlateral coordinates (e.g., the squared difference between
// the right hip coordinate value at the initial state and the left hip
// coordinate value at the final state).
class MocoSymmetryCost : public MocoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoSymmetryCost, MocoCost);
public:
    MocoSymmetryCost() = default;
    MocoSymmetryCost(std::string name) : MocoCost(std::move(name)) {}
    MocoSymmetryCost(std::string name, double weight)
            : MocoCost(std::move(name), weight) {}
protected:
    void calcCostImpl(const CostInput& input, SimTK::Real& cost)
            const override {
        // Initial states: coordinates - positions (all but pelvis tx)
        SimTK::Real position_pelvis_tilt_IS =
                m_coord_pelvis_tilt->getValue(input.initial_state);
        SimTK::Real position_pelvis_ty_IS =
                m_coord_pelvis_ty->getValue(input.initial_state);
        SimTK::Real position_hip_flexion_l_IS =
                m_coord_hip_flexion_l->getValue(input.initial_state);
        SimTK::Real position_hip_flexion_r_IS =
                m_coord_hip_flexion_r->getValue(input.initial_state);
        SimTK::Real position_knee_angle_l_IS =
                m_coord_knee_angle_l->getValue(input.initial_state);
        SimTK::Real position_knee_angle_r_IS =
                m_coord_knee_angle_r->getValue(input.initial_state);
        SimTK::Real position_ankle_angle_l_IS =
                m_coord_ankle_angle_l->getValue(input.initial_state);
        SimTK::Real position_ankle_angle_r_IS =
                m_coord_ankle_angle_r->getValue(input.initial_state);
        SimTK::Real position_lumbar_IS =
                m_coord_lumbar->getValue(input.initial_state);
        // Final states: coordinates - positions (all but pelvis tx)
        SimTK::Real position_pelvis_tilt_FS =
                m_coord_pelvis_tilt->getValue(input.final_state);
        SimTK::Real position_pelvis_ty_FS =
                m_coord_pelvis_ty->getValue(input.final_state);
        SimTK::Real position_hip_flexion_l_FS =
                m_coord_hip_flexion_l->getValue(input.final_state);
        SimTK::Real position_hip_flexion_r_FS =
                m_coord_hip_flexion_r->getValue(input.final_state);
        SimTK::Real position_knee_angle_l_FS =
                m_coord_knee_angle_l->getValue(input.final_state);
        SimTK::Real position_knee_angle_r_FS =
                m_coord_knee_angle_r->getValue(input.final_state);
        SimTK::Real position_ankle_angle_l_FS =
                m_coord_ankle_angle_l->getValue(input.final_state);
        SimTK::Real position_ankle_angle_r_FS =
                m_coord_ankle_angle_r->getValue(input.final_state);
        SimTK::Real position_lumbar_FS =
                m_coord_lumbar->getValue(input.final_state);
        // Cost
        cost = SimTK::square(position_pelvis_tilt_IS -
                    position_pelvis_tilt_FS) +
            SimTK::square(position_pelvis_ty_IS -
                    position_pelvis_ty_FS) +
            SimTK::square(position_hip_flexion_l_IS -
                    position_hip_flexion_r_FS) +
            SimTK::square(position_hip_flexion_r_IS -
                    position_hip_flexion_l_FS) +
            SimTK::square(position_knee_angle_l_IS -
                    position_knee_angle_r_FS) +
            SimTK::square(position_knee_angle_r_IS -
                    position_knee_angle_l_FS) +
            SimTK::square(position_ankle_angle_l_IS -
                    position_ankle_angle_r_FS) +
            SimTK::square(position_ankle_angle_r_IS -
                    position_ankle_angle_l_FS) +
            SimTK::square(position_lumbar_IS -
                    position_lumbar_FS);
    }
    void initializeOnModelImpl(const Model& model) const {
        // Coordinates
        m_coord_pelvis_tilt.reset(
                &model.getCoordinateSet().get("pelvis_tilt"));
        m_coord_pelvis_tx.reset(
                &model.getCoordinateSet().get("pelvis_tx"));
        m_coord_pelvis_ty.reset(
                &model.getCoordinateSet().get("pelvis_ty"));
        m_coord_hip_flexion_l.reset(
                &model.getCoordinateSet().get("hip_flexion_l"));
        m_coord_hip_flexion_r.reset(
                &model.getCoordinateSet().get("hip_flexion_r"));
        m_coord_knee_angle_l.reset(
                &model.getCoordinateSet().get("knee_angle_l"));
        m_coord_knee_angle_r.reset(
                &model.getCoordinateSet().get("knee_angle_r"));
        m_coord_ankle_angle_l.reset(
                &model.getCoordinateSet().get("ankle_angle_l"));
        m_coord_ankle_angle_r.reset(
                &model.getCoordinateSet().get("ankle_angle_r"));
        m_coord_lumbar.reset(
                &model.getCoordinateSet().get("lumbar"));
    }
    int getNumIntegralsImpl() const {return 0;};
private:
    // Coordinates
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_pelvis_tilt;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_pelvis_tx;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_pelvis_ty;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_hip_flexion_l;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_hip_flexion_r;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_knee_angle_l;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_knee_angle_r;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_ankle_angle_l;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_ankle_angle_r;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_lumbar;
};

// Set a coordinate tracking problem. Here the model is driven by muscles whose
// muscle-tendon lengths, velocities, and moment arms are derived from
// polynomial approximations of joint coordinates. The cost function combines
// a tracking term (coordinate values and speeds), an effort term (squared
// controls), and a term encouraging symmetry of the coordinate values over
// half a gait cycle.
void testCoordinateTracking_MusclePolynomials() {
    // Create a MocoTrack
    MocoTrack track;
    track.setName("coordinateTracking_MusclePolynomials");
    // Set model
    ModelProcessor modelprocessor = ModelProcessor(
            "gait10dof18musc_MusclePolynomials.osim");
    track.setModel(modelprocessor);
    // Set experimental coordinate values to track
    track.setStatesReference(TableProcessor("referenceCoordinate.sto") |
            TabOpLowPassFilter(6));
    track.set_states_global_tracking_weight(10.0);
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    // Set guess
    track.set_apply_tracked_states_to_guess(true);
    // Set time bounds
    track.set_initial_time(0.0);
    track.set_final_time(0.47008941);
    // Initialize study
    MocoStudy moco = track.initialize();
    // Set solver settings
    MocoCasADiSolver& solver = moco.updSolver<MocoCasADiSolver>();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(100000);
    solver.set_parallel(8);
    // Update problem
    MocoProblem& problem = moco.updProblem();
    // Add symmetry cost
    auto* symmetryCost = problem.addCost<MocoSymmetryCost>("symmetryCost");
    symmetryCost->set_weight(10);
    // Add effort cots
    auto* effortCost = problem.addCost<MocoControlCost>("effortCost");
    effortCost->set_weight(10);
    // Adjust bounds
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
            {-20*SimTK::Pi/180,-10*SimTK::Pi/180});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0,1});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
            {0.75,1.25});
    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
            {-50*SimTK::Pi/180,0});
    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
            {-50*SimTK::Pi/180,0});
    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/lumbar/lumbar/value",
            {0,20*SimTK::Pi/180});
    // Solve problem
    MocoSolution solution = moco.solve();
}

// Set a coordinate tracking problem. Here the model is driven by muscles whose
// paths are defined by GeometryPath. The cost function combines a tracking
// term (coordinate values and speeds), an effort term (squared controls), and
// a term encouraging symmetry of the coordinate values over half a gait cycle.
void testCoordinateTracking_MuscleGeometryPath() {
    // Create a MocoTrack
    MocoTrack track;
    track.setName("coordinateTracking_MuscleGeometryPath");
    // Set model
    ModelProcessor modelprocessor = ModelProcessor(
            "gait10dof18musc_MuscleGeometryPath.osim");
    track.setModel(modelprocessor);
    // Set experimental coordinate values to track
    track.setStatesReference(TableProcessor("referenceCoordinate.sto") |
            TabOpLowPassFilter(6));
    track.set_states_global_tracking_weight(10.0);
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    // Set guess
    track.set_apply_tracked_states_to_guess(true);
    // Set time bounds
    track.set_initial_time(0.0);
    track.set_final_time(0.47008941);
    // Initialize study
    MocoStudy moco = track.initialize();
    // Set solver settings
    MocoCasADiSolver& solver = moco.updSolver<MocoCasADiSolver>();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(100000);
    solver.set_parallel(8);
    // Update problem
    MocoProblem& problem = moco.updProblem();
    // Add symmetry cost
    auto* symmetryCost = problem.addCost<MocoSymmetryCost>("symmetryCost");
    symmetryCost->set_weight(10);
    // Add effort cots
    auto* effortCost = problem.addCost<MocoControlCost>("effortCost");
    effortCost->set_weight(10);
    // Adjust bounds
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
            {-20*SimTK::Pi/180,-10*SimTK::Pi/180});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0,1});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
            {0.75,1.25});
    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
            {-50*SimTK::Pi/180,0});
    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
            {-50*SimTK::Pi/180,0});
    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/lumbar/lumbar/value",
            {0,20*SimTK::Pi/180});
    // Solve problem
    MocoSolution solution = moco.solve();
}

// Set a coordinate tracking problem. Here the model is driven by coordinate
// actuators. The cost function combines a tracking term (coordinate values and
// speeds), an effort term (squared controls), and a term encouraging symmetry
// of the coordinate values over half a gait cycle.
void testCoordinateTracking_CoordinateActuators() {
    // Create a MocoTrack
    MocoTrack track;
    track.setName("coordinateTracking_CoordinateActuators");
    // Set model
    ModelProcessor modelprocessor = ModelProcessor(
            "gait10dof18musc_CoordinateActuators.osim");
    track.setModel(modelprocessor);
    // Set experimental coordinate values to track
    track.setStatesReference(TableProcessor("referenceCoordinate.sto") |
            TabOpLowPassFilter(6));
    track.set_states_global_tracking_weight(10.0);
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    // Set guess
    track.set_apply_tracked_states_to_guess(true);
    // Set time bounds
    track.set_initial_time(0.0);
    track.set_final_time(0.47008941);
    // Initialize study
    MocoStudy moco = track.initialize();
    // Set solver settings
    MocoCasADiSolver& solver = moco.updSolver<MocoCasADiSolver>();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(100000);
    solver.set_parallel(8);
    // Update problem
    MocoProblem& problem = moco.updProblem();
    // Add symmetry cost
    auto* symmetryCost = problem.addCost<MocoSymmetryCost>("symmetryCost");
    symmetryCost->set_weight(10);
    // Add effort cots
    auto* effortCost = problem.addCost<MocoControlCost>("effortCost");
    effortCost->set_weight(10);
    // Adjust bounds
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
            {-20*SimTK::Pi/180,-10*SimTK::Pi/180});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0,1});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
            {0.75,1.25});
    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
            {-50*SimTK::Pi/180,0});
    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
            {-50*SimTK::Pi/180,0});
    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/lumbar/lumbar/value",
            {0,20*SimTK::Pi/180});
    // Solve problem
    MocoSolution solution = moco.solve();
}

int main() {
   testCoordinateTracking_MusclePolynomials();
   testCoordinateTracking_MuscleGeometryPath();
   testCoordinateTracking_CoordinateActuators();
}
