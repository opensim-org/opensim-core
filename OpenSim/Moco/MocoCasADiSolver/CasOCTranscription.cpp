/* -------------------------------------------------------------------------- *
 * OpenSim Moco: CasOCTranscription.cpp                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 Stanford University and the Authors                     *
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
#include "CasOCTranscription.h"
#include <casadi/core/calculus.hpp>

using casadi::DM;
using casadi::MX;
using casadi::MXVector;
using casadi::Slice;

namespace CasOC {

// http://casadi.sourceforge.net/api/html/d7/df0/solvers_2callback_8py-example.html

/// This class allows us to observe intermediate iterates throughout the
/// optimization.
class NlpsolCallback : public casadi::Callback {
public:
    NlpsolCallback(const Transcription& transcription, const Problem& problem,
            casadi_int numVariables, casadi_int numConstraints,
            casadi_int outputInterval)
            : m_transcription(transcription), m_problem(problem),
              m_numVariables(numVariables), m_numConstraints(numConstraints),
              m_callbackInterval(outputInterval) {
        construct("NlpsolCallback", {});
    }
    casadi_int get_n_in() override { return casadi::nlpsol_n_out(); }
    casadi_int get_n_out() override { return 1; }
    std::string get_name_in(casadi_int i) override {
        return casadi::nlpsol_out(i);
    }
    std::string get_name_out(casadi_int) override { return "ret"; }
    casadi::Sparsity get_sparsity_in(casadi_int i) override {
        auto n = casadi::nlpsol_out(i);
        if (n == "f") {
            return casadi::Sparsity::scalar();
        } else if (n == "x" || n == "lam_x") {
            return casadi::Sparsity::dense(m_numVariables, 1);
        } else if (n == "g" || n == "lam_g") {
            return casadi::Sparsity::dense(m_numConstraints, 1);
        } else {
            return casadi::Sparsity(0, 0);
        }
    }
    std::vector<DM> eval(const std::vector<DM>& args) const override {
        if (m_callbackInterval > 0 && evalCount % m_callbackInterval == 0) {
            Iterate iterate = m_problem.createIterate<Iterate>();
            iterate.variables = m_transcription.unscaleVariables(
                    m_transcription.expandVariables(args.at(0)));
            m_transcription.calcInterpolatingControls(iterate.variables);
            iterate.iteration = evalCount;
            iterate.times =
                    m_transcription.createTimes(iterate.variables[initial_time],
                            iterate.variables[final_time]);
            m_problem.intermediateCallbackWithIterate(iterate);
        }
        m_problem.intermediateCallback();
        ++evalCount;
        return {0};
    }

private:
    const Transcription& m_transcription;
    const Problem& m_problem;
    casadi_int m_numVariables;
    casadi_int m_numConstraints;
    casadi_int m_callbackInterval;
    mutable int evalCount = 0;
};

void Transcription::createVariablesAndSetBounds(const casadi::DM& grid,
        int numDefectsPerMeshInterval,
        int numPointsPerMeshInterval) {

    // Set the grid.
    // -------------
    // The grid for a transcription scheme includes both mesh points (i.e.
    // points that lie on the endpoints of a mesh interval) and any
    // additional collocation points that may lie on mesh interior (as in
    // Hermite-Simpson collocation, etc.).
    m_numMeshPoints = (int)m_solver.getMesh().size();
    m_numGridPoints = (int)grid.numel();
    m_numMeshIntervals = m_numMeshPoints - 1;
    m_numMeshInteriorPoints = m_numGridPoints - m_numMeshPoints;
    m_numDefectsPerMeshInterval = numDefectsPerMeshInterval;    
    m_numPointsPerMeshInterval = numPointsPerMeshInterval;
    m_numMultibodyResiduals = m_problem.isDynamicsModeImplicit()
                             ? m_problem.getNumMultibodyDynamicsEquations()
                             : 0;
    m_numAuxiliaryResiduals = m_problem.getNumAuxiliaryResidualEquations();
    m_numProjectionStates = m_problem.getNumProjectionConstraintEquations();

    m_numUDotErrorPoints = m_problem.isKinematicConstraintMethodBordalba2023()
            ? m_numGridPoints
            : m_numMeshPoints;
    // Dynamics constraints.
    m_numConstraints =
            m_numDefectsPerMeshInterval * m_numMeshIntervals +
            m_numMultibodyResiduals * m_numGridPoints +
            m_numAuxiliaryResiduals * m_numGridPoints +
            m_problem.getNumQErr() * m_numMeshPoints +
            m_problem.getNumUErr() * m_numMeshPoints +
            m_problem.getNumUDotErr() * m_numUDotErrorPoints +
            m_problem.getNumProjectionConstraintEquations() * m_numMeshIntervals;
    // Time constraints.
    m_numConstraints += 2 * m_numMeshIntervals;
    // Parameter constraints.
    m_numConstraints += m_problem.getNumParameters() * m_numMeshIntervals;
    // Endpoint constraints.
    m_constraints.endpoint.resize(
            m_problem.getEndpointConstraintInfos().size());
    for (int iec = 0; iec < (int)m_constraints.endpoint.size(); ++iec) {
        const auto& info = m_problem.getEndpointConstraintInfos()[iec];
        m_numConstraints += info.num_outputs;
    }
    // Path constraints.
    m_constraints.path.resize(m_problem.getPathConstraintInfos().size());
    m_numPathConstraintPoints =
            m_solver.getEnforcePathConstraintMeshInteriorPoints()
            ? m_numGridPoints : m_numMeshPoints;
    for (int ipc = 0; ipc < (int)m_constraints.path.size(); ++ipc) {
        const auto& info = m_problem.getPathConstraintInfos()[ipc];
        m_numConstraints += info.size() * m_numPathConstraintPoints;
    }
    m_grid = grid;

    // Define indices.
    // ---------------
    m_meshIndicesMap = createMeshIndices();
    std::vector<int> meshIndicesVector;
    std::vector<int> meshInteriorIndicesVector;
    std::vector<int> projectionStateIndicesVector;
    std::vector<int> notProjectionStateIndicesVector;
    for (int i = 0; i < m_meshIndicesMap.size2(); ++i) {
        if (m_meshIndicesMap(i).scalar() == 1) {
            meshIndicesVector.push_back(i);
            if (i == 0) {
                notProjectionStateIndicesVector.push_back(i);
            } else {
                projectionStateIndicesVector.push_back(i);
            }
        } else {
            meshInteriorIndicesVector.push_back(i);
            notProjectionStateIndicesVector.push_back(i);
        }
    }

    auto controlIndicesMap = createControlIndices();
    std::vector<int> controlIndicesVector;
    for (int i = 0; i < controlIndicesMap.size2(); ++i) {
        if (controlIndicesMap(i).scalar() == 1) {
            controlIndicesVector.push_back(i);
        }
    }
    m_numControlPoints = static_cast<int>(controlIndicesVector.size());

    auto makeTimeIndices = [](const std::vector<int>& in) {
        casadi::Matrix<casadi_int> out(1, in.size());
        for (int i = 0; i < (int)in.size(); ++i) { out(i) = in[i]; }
        return out;
    };

    std::vector<int> gridIndicesVector(m_numGridPoints);
    std::iota(gridIndicesVector.begin(), gridIndicesVector.end(), 0);
    m_gridIndices = makeTimeIndices(gridIndicesVector);
    m_meshIndices = makeTimeIndices(meshIndicesVector);
    m_meshInteriorIndices =
            makeTimeIndices(meshInteriorIndicesVector);
    m_pathConstraintIndices =
            m_solver.getEnforcePathConstraintMeshInteriorPoints()
            ? makeTimeIndices(gridIndicesVector)
            : makeTimeIndices(meshIndicesVector);
    m_controlIndices = makeTimeIndices(controlIndicesVector);
    m_projectionStateIndices = makeTimeIndices(projectionStateIndicesVector);
    m_notProjectionStateIndices =
            makeTimeIndices(notProjectionStateIndicesVector);

    // Create variables.
    // -----------------
    // Parameter variables.
    for (int imesh = 0; imesh < m_numMeshPoints; ++imesh) {
        m_scaledVectorVars[initial_time].push_back(
                MX::sym(fmt::format("initial_time_{}", imesh), 1, 1));
        m_scaledVectorVars[final_time].push_back(
                MX::sym(fmt::format("final_time_{}", imesh), 1, 1));
        m_scaledVectorVars[parameters].push_back(
                MX::sym(fmt::format("parameters_{}", imesh),
                        m_problem.getNumParameters(), 1));
    }
    // Trajectory variables.
    for (int igrid = 0; igrid < m_numGridPoints; ++igrid) {
        m_scaledVectorVars[states].push_back(
                MX::sym(fmt::format("states_{}", igrid),
                        m_problem.getNumStates(), 1));

        if (m_solver.getInterpolateControlMeshInteriorPoints()) {
            auto it = std::find(controlIndicesVector.begin(),
                    controlIndicesVector.end(), igrid);
            if (it != controlIndicesVector.end()) {
                m_scaledVectorVars[controls].push_back(
                        MX::sym(fmt::format("controls_{}", igrid),
                                m_problem.getNumControls(), 1));
            } else {
                m_scaledVectorVars[controls].push_back(
                        MX(m_problem.getNumControls(), 1));
            }
        } else {
            m_scaledVectorVars[controls].push_back(
                MX::sym(fmt::format("controls_{}", igrid),
                        m_problem.getNumControls(), 1));
        }

        m_scaledVectorVars[multipliers].push_back(
                MX::sym(fmt::format("multipliers_{}", igrid),
                        m_problem.getNumMultipliers(), 1));
        m_scaledVectorVars[derivatives].push_back(
                MX::sym(fmt::format("derivatives_{}", igrid),
                        m_problem.getNumDerivatives(), 1));
    }
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        m_scaledVectorVars[projection_states].push_back(MX::sym(
                fmt::format("projection_states_{}", imesh), 
                m_numProjectionStates, 1));
    }

    
    if (m_problem.isKinematicConstraintMethodBordalba2023()) {
        // In the projection method for enforcing kinematic constraints, the
        // slack variables are applied at the mesh points at the end of each
        // mesh interval.
        for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
            m_scaledVectorVars[slacks].push_back(
                    MX::sym(fmt::format("slacks_{}", imesh), 
                            m_problem.getNumSlacks(), 1));
        }
    } else {
        // In the Posa et al. 2016 method for enforcing kinematic constraints,
        // the mesh interior points will be the mesh interval midpoints in the
        // Hermite-Simpson collocation scheme.
        for (int imesh = 0; imesh < m_numMeshInteriorPoints; ++imesh) {
            m_scaledVectorVars[slacks].push_back(
                    MX::sym(fmt::format("slacks_{}", imesh),
                            m_problem.getNumSlacks(), 1));
        }
    }

    // Concatenate variables.
    m_scaledVars[initial_time] = MX::horzcat(m_scaledVectorVars[initial_time]);
    m_scaledVars[final_time] = MX::horzcat(m_scaledVectorVars[final_time]);
    m_scaledVars[parameters] = MX::horzcat(m_scaledVectorVars[parameters]);
    m_scaledVars[states] = MX::horzcat(m_scaledVectorVars[states]);
    m_scaledVars[projection_states] = 
            MX::horzcat(m_scaledVectorVars[projection_states]);
    m_scaledVars[controls] = MX::horzcat(m_scaledVectorVars[controls]);
    m_scaledVars[multipliers] = MX::horzcat(m_scaledVectorVars[multipliers]);
    m_scaledVars[derivatives] = MX::horzcat(m_scaledVectorVars[derivatives]);
    m_scaledVars[slacks] = MX::horzcat(m_scaledVectorVars[slacks]);

    // Interplate controls.
    calcInterpolatingControls(m_scaledVars);

    // Each vector contains MX matrix elements (states or state derivatives)
    // needed to construct the defect constraints for an individual mesh
    // interval.
    m_statesByMeshInterval = MXVector(m_numMeshIntervals);
    m_stateDerivativesByMeshInterval = MXVector(m_numMeshIntervals);
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        m_statesByMeshInterval[imesh] =
                MX(m_problem.getNumStates(), m_numPointsPerMeshInterval);
        m_stateDerivativesByMeshInterval[imesh] =
                MX(m_problem.getNumStates(), m_numPointsPerMeshInterval);
    }
    m_projectionStateDistances = MX(m_numProjectionStates, m_numMeshIntervals);

    // Set variable bounds.
    // --------------------
    auto initializeBoundsDM = [&](VariablesDM& bounds) {
        for (auto& kv : m_scaledVars) {
            bounds[kv.first] = DM(kv.second.rows(), kv.second.columns());
        }
    };
    initializeBoundsDM(m_lowerBounds);
    initializeBoundsDM(m_upperBounds);

    // The VariablesDM for scaling have length 1 in the time dimension.
    auto initializeScalingDM = [&](VariablesDM& bounds) {
        for (auto& kv : m_scaledVars) {
            bounds[kv.first] = DM(casadi::Sparsity::dense(kv.second.rows(), 1));
        }
    };

    initializeScalingDM(m_shift);
    initializeScalingDM(m_scale);

    Bounds infBounds = {-casadi::inf, casadi::inf};
    setVariableBounds(initial_time, 0, 0, m_problem.getTimeInitialBounds());
    setVariableBounds(final_time, 0, 0, m_problem.getTimeFinalBounds());
    setVariableBounds(initial_time, 0, Slice(1, m_numMeshPoints), infBounds);
    setVariableBounds(final_time, 0, Slice(1, m_numMeshPoints), infBounds);

    setVariableScaling(initial_time, 0, Slice(),
            m_problem.getTimeInitialBounds());
    setVariableScaling(final_time, 0, Slice(),
            m_problem.getTimeFinalBounds());

    {
        const auto& stateInfos = m_problem.getStateInfos();
        int is = 0;
        for (const auto& info : stateInfos) {
            setVariableBounds(
                    states, is, Slice(1, m_numGridPoints - 1), info.bounds);
            // The "0" grabs the first column (first mesh point).
            setVariableBounds(states, is, 0, info.initialBounds);
            // The "-1" grabs the last column (last mesh point).
            setVariableBounds(states, is, -1, info.finalBounds);
            setVariableScaling(states, is, Slice(), info.bounds);
            ++is;
        }
    }
    {
        // TODO update this to set bounds properly based on the control points.
        const auto& controlInfos = m_problem.getControlInfos();
        int ic = 0;
        for (const auto& info : controlInfos) {
            setVariableBounds(controls, ic, Slice(1, m_numGridPoints - 1),
                    info.bounds);
            setVariableBounds(controls, ic, 0, info.initialBounds);
            setVariableBounds(controls, ic, -1, info.finalBounds);
            setVariableScaling(controls, ic, Slice(), info.bounds);
            ++ic;
        }
    }
    {
        const auto& multiplierInfos = m_problem.getMultiplierInfos();
        int im = 0;
        for (const auto& info : multiplierInfos) {
            setVariableBounds(multipliers, im, Slice(1, m_numGridPoints - 1),
                    info.bounds);
            setVariableBounds(multipliers, im, 0, info.initialBounds);
            setVariableBounds(multipliers, im, -1, info.finalBounds);
            setVariableScaling(multipliers, im, Slice(), info.bounds);
            ++im;
        }
    }
    {
        if (m_problem.isDynamicsModeImplicit()) {
            // "Slice()" grabs everything in that dimension (like ":" in
            // Matlab).
            setVariableBounds(derivatives, Slice(0, m_problem.getNumSpeeds()),
                    Slice(), m_solver.getImplicitMultibodyAccelerationBounds());
            setVariableScaling(derivatives, Slice(0, m_problem.getNumSpeeds()),
                    Slice(), m_solver.getImplicitMultibodyAccelerationBounds());

        }
        if (m_problem.getNumAuxiliaryResidualEquations()) {
            setVariableBounds(derivatives,
                    Slice(m_problem.getNumAccelerations(),
                            m_problem.getNumDerivatives()),
                    Slice(), m_solver.getImplicitAuxiliaryDerivativeBounds());
            setVariableScaling(derivatives,
                    Slice(m_problem.getNumAccelerations(),
                            m_problem.getNumDerivatives()),
                    Slice(), m_solver.getImplicitAuxiliaryDerivativeBounds());
        }
    }
    {
        const auto& slackInfos = m_problem.getSlackInfos();
        int isl = 0;
        for (const auto& info : slackInfos) {
            setVariableBounds(slacks, isl, Slice(), info.bounds);
            setVariableScaling(slacks, isl, Slice(), info.bounds);
            ++isl;
        }
    }
    {
        // Use the bounds from the "true" multibody states to set the bounds for
        // the projection states.
        const auto& stateInfos = m_problem.getStateInfos();
        for (int ips = 0; ips < m_numProjectionStates; ++ips) {
            const auto& info = stateInfos[ips];
            setVariableBounds(
                    projection_states, ips, Slice(0, m_numMeshIntervals - 1),
                    info.bounds);
            setVariableBounds(projection_states, ips, -1, info.finalBounds);
            setVariableScaling(projection_states, ips, Slice(), info.bounds);
        }
    }
    {
        const auto& paramInfos = m_problem.getParameterInfos();
        int ip = 0;
        for (const auto& info : paramInfos) {
            setVariableBounds(parameters, ip, Slice(), info.bounds);
            setVariableScaling(parameters, ip, Slice(), info.bounds);
            ++ip;
        }
    }
    m_unscaledVars = unscaleVariables(m_scaledVars);

    // Convert parameter variables to trajectories of length m_numGridPoints so
    // we can easily index over them (e.g., in evalOnTrajectory()). We have
    // parameter values for each mesh point, and they are enforced to be 
    // time-invariant via constraints in calcDefects(). To maintain optimal 
    // sparsity structure, parameter values in the trajectories will be assigned
    // based on the parameter variable at the corresponding mesh interval.
    const auto& mesh = m_solver.getMesh();
    m_intervals = MX(casadi::Sparsity::dense(1, m_numMeshIntervals));
    m_times = MX(casadi::Sparsity::dense(1, m_numGridPoints));
    m_parameters = MX(casadi::Sparsity::dense(
            m_problem.getNumParameters(), m_numGridPoints));
    int N = m_numPointsPerMeshInterval - 1;
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        m_intervals(imesh) = (mesh[imesh + 1] - mesh[imesh]) * 
                (m_unscaledVars[final_time](imesh) - 
                 m_unscaledVars[initial_time](imesh)); 

        int igrid = imesh * N;
        for (int i = 0; i < N; ++i) {
            m_parameters(Slice(), igrid + i) =
                    m_unscaledVars[parameters](Slice(), imesh);
            m_times(igrid + i) = (m_unscaledVars[final_time](imesh) -
                                  m_unscaledVars[initial_time](imesh)) *
                                        m_grid(igrid + i) +
                                  m_unscaledVars[initial_time](imesh);
        }
    }
    m_parameters(Slice(), -1) = m_unscaledVars[parameters](Slice(), -1);
    m_times(-1) = (m_unscaledVars[final_time](-1) -
                   m_unscaledVars[initial_time](-1)) *
                        m_grid(-1) +
                   m_unscaledVars[initial_time](-1);

    m_duration = m_unscaledVars[final_time] - m_unscaledVars[initial_time];

    // Create MXVector containing states for calcDefects().
    convertStatesOrStateDerivativesToMXVector(m_unscaledVars[states],
            m_unscaledVars[projection_states], m_statesByMeshInterval);
}

void Transcription::transcribe() {

    // Cost.
    // =====
    setObjectiveAndEndpointConstraints();

    // Compute DAEs at necessary grid points.
    // ======================================
    const int NQ = m_problem.getNumCoordinates();
    const int NU = m_problem.getNumSpeeds();
    const int NS = m_problem.getNumStates();
    OPENSIM_THROW_IF(NQ != NU, OpenSim::Exception,
            "Problems with differing numbers of coordinates and speeds are "
            "not supported (e.g., quaternions).");

    // Initialize memory for time constraints.
    // ---------------------------------------
    m_constraints.time = MX(casadi::Sparsity::dense(2, m_numMeshIntervals));
    m_constraintsLowerBounds.time = DM::zeros(2, m_numMeshIntervals);
    m_constraintsUpperBounds.time = DM::zeros(2, m_numMeshIntervals);

    // Initialize memory for parameter constraints.
    // --------------------------------------------
    m_constraints.parameters = MX(casadi::Sparsity::dense(
            m_problem.getNumParameters(), m_numMeshIntervals));
    m_constraintsLowerBounds.parameters = 
            DM::zeros(m_problem.getNumParameters(), m_numMeshIntervals);
    m_constraintsUpperBounds.parameters = 
            DM::zeros(m_problem.getNumParameters(), m_numMeshIntervals);

    // Initialize memory for state derivatives and defects.
    // ----------------------------------------------------
    m_xdot = MX(NS, m_numGridPoints);
    m_xdot_projection = MX(m_numProjectionStates, m_numMeshIntervals);
    m_constraints.defects = MX(casadi::Sparsity::dense(
            m_numDefectsPerMeshInterval, m_numMeshIntervals));
    m_constraintsLowerBounds.defects =
            DM::zeros(m_numDefectsPerMeshInterval, m_numMeshIntervals);
    m_constraintsUpperBounds.defects =
            DM::zeros(m_numDefectsPerMeshInterval, m_numMeshIntervals);

    // Initialize memory for implicit multibody residuals.
    // ---------------------------------------------------
    m_constraints.multibody_residuals = MX(casadi::Sparsity::dense(
            m_numMultibodyResiduals, m_numGridPoints));
    m_constraintsLowerBounds.multibody_residuals =
            DM::zeros(m_numMultibodyResiduals, m_numGridPoints);
    m_constraintsUpperBounds.multibody_residuals =
            DM::zeros(m_numMultibodyResiduals, m_numGridPoints);

    // Initialize memory for implicit auxiliary residuals.
    // ---------------------------------------------------
    m_constraints.auxiliary_residuals = MX(casadi::Sparsity::dense(
            m_numAuxiliaryResiduals, m_numGridPoints));
    m_constraintsLowerBounds.auxiliary_residuals =
            DM::zeros(m_numAuxiliaryResiduals, m_numGridPoints);
    m_constraintsUpperBounds.auxiliary_residuals =
            DM::zeros(m_numAuxiliaryResiduals, m_numGridPoints);

    // Initialize memory for kinematic constraints.
    // --------------------------------------------
    int nqerr = m_problem.getNumQErr();
    int nuerr = m_problem.getNumUErr();
    int nudoterr = m_problem.getNumUDotErr();
    int nkc = m_problem.getNumKinematicConstraintEquations();

    const auto& kcBounds = m_problem.getKinematicConstraintBounds();
    if (m_problem.isKinematicConstraintMethodBordalba2023()) {
        m_constraints.kinematic = MX(casadi::Sparsity::dense(
                nqerr + nuerr, m_numMeshPoints));
        m_constraintsLowerBounds.kinematic = casadi::DM::repmat(
                kcBounds.lower, nqerr + nuerr, m_numMeshPoints);
        m_constraintsUpperBounds.kinematic = casadi::DM::repmat(
                kcBounds.upper, nqerr + nuerr, m_numMeshPoints);

        m_constraints.kinematic_udoterr = MX(casadi::Sparsity::dense(
                nudoterr, m_numUDotErrorPoints));
        m_constraintsLowerBounds.kinematic_udoterr = casadi::DM::repmat(
                kcBounds.lower, nudoterr, m_numUDotErrorPoints);
        m_constraintsUpperBounds.kinematic_udoterr = casadi::DM::repmat(
                kcBounds.upper, nudoterr, m_numUDotErrorPoints);
    } else {
        m_constraints.kinematic = MX(casadi::Sparsity::dense(
                nkc, m_numMeshPoints));
        m_constraintsLowerBounds.kinematic = casadi::DM::repmat(
                kcBounds.lower, nkc, m_numMeshPoints);
        m_constraintsUpperBounds.kinematic = casadi::DM::repmat(
                kcBounds.upper, nkc, m_numMeshPoints);
    }

    // Initialize memory for projection constraints.
    // ---------------------------------------------
    int numProjectionConstraints =
            m_problem.getNumProjectionConstraintEquations();
    m_constraints.projection = MX(
            casadi::Sparsity::dense(numProjectionConstraints,
                                    m_numMeshIntervals));
    m_constraintsLowerBounds.projection =
            DM::zeros(numProjectionConstraints, m_numMeshIntervals);
    m_constraintsUpperBounds.projection =
            DM::zeros(numProjectionConstraints, m_numMeshIntervals);

    // qdot
    // ----
    const MX u = m_unscaledVars[states](Slice(NQ, NQ + NU), Slice());
    m_xdot(Slice(0, NQ), Slice()) = u;
    if (m_numProjectionStates) {
        const MX u_projection = m_unscaledVars[projection_states]
                            (Slice(NQ, NQ + NU), Slice());
        m_xdot_projection(Slice(0, NQ), Slice()) = u_projection;
    }

    // projection constraints
    // ----------------------
    if (m_problem.getNumKinematicConstraintEquations() &&
            !m_problem.isPrescribedKinematics() &&
            m_problem.getEnforceConstraintDerivatives()) {

        OPENSIM_THROW_IF(!m_problem.isKinematicConstraintMethodBordalba2023() &&
                         m_solver.getTranscriptionScheme() != "hermite-simpson",
            OpenSim::Exception,
            "Expected the 'hermite-simpson' transcription scheme when using "
            "the Posa et al. 2016 method for enforcing kinematic constraints, "
            "but the '{}' scheme was selected.",
            m_solver.getTranscriptionScheme());

        if (m_solver.getTranscriptionScheme() == "hermite-simpson" &&
                !m_problem.isKinematicConstraintMethodBordalba2023()) {
            // In Hermite-Simpson, we must compute a velocity correction at all
            // mesh interval midpoints and update qdot.
            // See MocoCasADiVelocityCorrection for more details. This function
            // only takes multibody state variables: coordinates and speeds.
            const auto velocityCorrectionOut = evalOnTrajectory(
                    m_problem.getVelocityCorrection(),
                    {multibody_states, slacks},
                    m_meshInteriorIndices);
            const auto u_correction = velocityCorrectionOut.at(0);

            m_xdot(Slice(0, NQ), m_meshInteriorIndices) += u_correction;
        }

        if (m_numProjectionStates) {
            const auto projectionOut = evalOnTrajectory(
                    m_problem.getStateProjection(),
                    {multibody_states, slacks},
                    m_projectionStateIndices);
            const auto x_proj = projectionOut.at(0);

            const MX x = m_unscaledVars[states](Slice(0, NQ + NU),
                    m_projectionStateIndices);
            const MX x_prime = m_unscaledVars[projection_states];

            // Bordalba et al. (2023) projection equation: x = x' + F_x^T mu.
            m_constraints.projection = x - x_prime - x_proj;
        }
    }

    // udot, zdot, residual, kcerr
    // ---------------------------
    if (m_problem.isDynamicsModeImplicit()) {
        // udot.
        const MX w = m_unscaledVars[derivatives]
                     (Slice(0, m_problem.getNumSpeeds()), Slice());
        m_xdot(Slice(NQ, NQ + NU), Slice()) = w;
        if (m_numProjectionStates) {
            const MX w_projection = m_unscaledVars[derivatives]
                                    (Slice(0, m_problem.getNumSpeeds()),
                                            m_projectionStateIndices);
            m_xdot_projection(Slice(NQ, NQ + NU), Slice()) = w_projection;
        }

        std::vector<Var> inputs{states, controls, multipliers, derivatives};

        // When the model has kinematic constraints, we must treat grid points
        // differently, as kinematic constraints are computed for only some
        // grid points. When the model does *not* have kinematic constraints,
        // the DAE is the same for all grid points, but the evaluation is still
        // done separately to keep implementation general.

        // residual, zdot, kcerr
        // Points where we compute algebraic constraints.
        {
            const auto out =
                    evalOnTrajectory(m_problem.getImplicitMultibodySystem(),
                            inputs, m_meshIndices);
            m_constraints.multibody_residuals(Slice(), m_meshIndices) =
                    out.at(0);
            // zdot.
            m_xdot(Slice(NQ + NU, NS), m_meshIndices) = out.at(1);
            m_constraints.auxiliary_residuals(Slice(), m_meshIndices) =
                    out.at(2);
            if (m_problem.isKinematicConstraintMethodBordalba2023()) {
                m_constraints.kinematic(Slice(0, nqerr), Slice()) = out.at(3)(
                        Slice(0, nqerr), Slice());
                m_constraints.kinematic(Slice(nqerr, nqerr + nuerr), Slice())
                        = out.at(3)(Slice(nqerr, nqerr + nuerr), Slice());
                m_constraints.kinematic_udoterr(Slice(), m_meshIndices) =
                        out.at(3)(
                                Slice(nqerr + nuerr, nqerr + nuerr + nudoterr),
                                Slice());
            } else {
                m_constraints.kinematic = out.at(3);
            }
        }

        // Points where we ignore algebraic constraints.
        if (m_numMeshInteriorPoints) {
            const casadi::Function& implicitMultibodyFunction =
                    m_problem.isKinematicConstraintMethodBordalba2023() ?
                    m_problem.getImplicitMultibodySystem() :
                    m_problem.getImplicitMultibodySystemIgnoringConstraints();
            const auto out = evalOnTrajectory(implicitMultibodyFunction, inputs,
                    m_meshInteriorIndices);
            m_constraints.multibody_residuals(Slice(), m_meshInteriorIndices) =
                    out.at(0);
            // zdot.
            m_xdot(Slice(NQ + NU, NS), m_meshInteriorIndices) =
                    out.at(1);
            m_constraints.auxiliary_residuals(Slice(), m_meshInteriorIndices) =
                    out.at(2);
            if (m_problem.isKinematicConstraintMethodBordalba2023()) {
                m_constraints.kinematic_udoterr(Slice(), m_meshInteriorIndices)
                        = out.at(3)(
                                Slice(nqerr + nuerr, nqerr + nuerr + nudoterr),
                                Slice());
            }
        }

        // Points where the multibody residuals depend on the projection states.
        if (m_numProjectionStates) {
            const casadi::Function& implicitMultibodyFunction =
                    m_problem.getImplicitMultibodySystem();
            const auto out = evalOnTrajectory(implicitMultibodyFunction, 
                    {projection_states, controls, multipliers, derivatives},
                    m_projectionStateIndices);
            // This overrides the previous function evaluation assignments for
            // `kinematic_udoterr` and `multibody_residuals` at the mesh indices
            // (i.e., for "points where we compute algebraic constraints"). We 
            // still need the function evaluations above since we need state
            // derivatives for auxiliary states on the mesh points (note that 
            // the projection states overlap with all the mesh indices except 
            // the first mesh point). This also keeps the implementation above
            // general when we do not have projection states.
            m_constraints.multibody_residuals(
                    Slice(), m_projectionStateIndices) = out.at(0);
            m_constraints.kinematic_udoterr(Slice(), m_projectionStateIndices)
                    = out.at(3)(Slice(nqerr + nuerr, nqerr + nuerr + nudoterr),
                            Slice());
        }

    } else { // Explicit dynamics mode.
        std::vector<Var> inputs{states, controls, multipliers, derivatives};

        // udot, zdot, kcerr.
        // Points where we compute algebraic constraints.
        {
            // Evaluate the multibody system function and get udot
            // (speed derivatives) and zdot (auxiliary derivatives).
            const auto out = evalOnTrajectory(
                    m_problem.getMultibodySystem(), inputs, m_meshIndices);
            m_xdot(Slice(NQ, NQ + NU), m_meshIndices) = out.at(0);
            m_xdot(Slice(NQ + NU, NS), m_meshIndices) = out.at(1);
            m_constraints.auxiliary_residuals(Slice(), m_meshIndices) =
                    out.at(2);
            if (m_problem.isKinematicConstraintMethodBordalba2023()) {
                m_constraints.kinematic(Slice(0, nqerr), Slice()) = out.at(3)(
                        Slice(0, nqerr), Slice());
                m_constraints.kinematic(Slice(nqerr, nqerr + nuerr), Slice())
                        = out.at(3)(Slice(nqerr, nqerr + nuerr), Slice());
                m_constraints.kinematic_udoterr(Slice(), m_meshIndices) =
                        out.at(3)(
                                Slice(nqerr + nuerr, nqerr + nuerr + nudoterr),
                                Slice());
            } else {
                m_constraints.kinematic = out.at(3);
            }
        }

        // Points where we ignore algebraic constraints.
        if (m_numMeshInteriorPoints) {
            // In the Bordalba et al. 2023 method, we need to enforce the 
            // acceleration-level kinematic constraints at the mesh interval
            // interior points.
            const casadi::Function& multibodyFunction =
                    m_problem.isKinematicConstraintMethodBordalba2023() ?
                    m_problem.getMultibodySystem() :
                    m_problem.getMultibodySystemIgnoringConstraints();
            const auto out = evalOnTrajectory(multibodyFunction, inputs,
                    m_meshInteriorIndices);
            m_xdot(Slice(NQ, NQ + NU), m_meshInteriorIndices) =
                    out.at(0);
            m_xdot(Slice(NQ + NU, NS), m_meshInteriorIndices) =
                    out.at(1);
            m_constraints.auxiliary_residuals(Slice(), m_meshInteriorIndices) =
                    out.at(2);
            if (m_problem.isKinematicConstraintMethodBordalba2023()) {
                m_constraints.kinematic_udoterr(Slice(), m_meshInteriorIndices)
                        = out.at(3)(
                                Slice(nqerr + nuerr, nqerr + nuerr + nudoterr),
                                Slice());
            }
        }

        // Points where the state derivatives depend on the projection states.
        if (m_numProjectionStates) {
            const auto out = evalOnTrajectory(m_problem.getMultibodySystem(),
                    {projection_states, controls, multipliers, derivatives},
                    m_projectionStateIndices);
            // This state derivative is not used by Legendre-Gauss collocation
            // since there is no collocation point at mesh interval endpoint.
            // However, we need this function evaluation, since we still enforce
            // the acceleration-level kinematic constraints at the mesh interval
            // endpoints.
            m_xdot_projection(Slice(NQ, NQ + NU), Slice()) = out.at(0);
            // This overrides the previous function evaluation assignments for
            // `kinematic_udoterr` at the mesh indices (i.e., for "points where
            // we compute algebraic constraints"). We still need the function 
            // evaluations above since we need state derivatives for auxiliary 
            // states on the mesh points (note that the projection states 
            // overlap with all the mesh indices except the first mesh point). 
            // This also keeps the implementation above general when we do not 
            // have projection states.
            m_constraints.kinematic_udoterr(Slice(), m_projectionStateIndices)
                    = out.at(3)(Slice(nqerr + nuerr, nqerr + nuerr + nudoterr),
                            Slice());
        }
    }

    // Fill MXVector with state derivatives for calcDefects().
    convertStatesOrStateDerivativesToMXVector(m_xdot, m_xdot_projection, 
            m_stateDerivativesByMeshInterval);

    // Calculate defects.
    // ------------------
    calcDefects();

    // Time and parameter constraints.
    // ------------------------------
    const auto& ti = m_unscaledVars.at(initial_time);
    const auto& tf = m_unscaledVars.at(final_time);
    const auto& p = m_unscaledVars.at(parameters);
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        m_constraints.time(0, imesh) = ti(imesh + 1) - ti(imesh);
        m_constraints.time(1, imesh) = tf(imesh + 1) - tf(imesh);
        m_constraints.parameters(Slice(), imesh) = 
                p(Slice(), imesh + 1) - p(Slice(), imesh);
    }

    // Path constraints
    // ----------------
    // The individual path constraint functions are passed to CasADi to
    // maximize CasADi's ability to take derivatives efficiently.
    int numPathConstraints = (int)m_problem.getPathConstraintInfos().size();
    m_constraints.path.resize(numPathConstraints);
    m_constraintsLowerBounds.path.resize(numPathConstraints);
    m_constraintsUpperBounds.path.resize(numPathConstraints);
    for (int ipc = 0; ipc < (int)m_constraints.path.size(); ++ipc) {
        const auto& info = m_problem.getPathConstraintInfos()[ipc];
        const auto out = evalOnTrajectory(*info.function,
                {states, controls, multipliers, derivatives},
                m_pathConstraintIndices);
        m_constraints.path[ipc] = out.at(0);
        m_constraintsLowerBounds.path[ipc] =
                casadi::DM::repmat(info.lowerBounds, 1,
                                   m_numPathConstraintPoints);
        m_constraintsUpperBounds.path[ipc] =
                casadi::DM::repmat(info.upperBounds, 1,
                                   m_numPathConstraintPoints);
    }
}

void Transcription::setObjectiveAndEndpointConstraints() {
    DM quadCoeffs = this->createQuadratureCoefficients();

    // Objective.
    // ----------
    m_objectiveTermNames.clear();
    for (int ic = 0; ic < m_problem.getNumCosts(); ++ic) {
        const auto& info = m_problem.getCostInfos()[ic];
        m_objectiveTermNames.push_back(info.name);
    }
    bool minimizeLagrangeMultipliers =
            m_solver.getMinimizeLagrangeMultipliers() &&
            m_problem.getNumMultipliers();
    if (minimizeLagrangeMultipliers) {
        m_objectiveTermNames.push_back("multipliers");
    }
    bool minimizeAccelerations =
            m_solver.getMinimizeImplicitMultibodyAccelerations() &&
            m_problem.isDynamicsModeImplicit();
    if (minimizeAccelerations) {
        m_objectiveTermNames.push_back("accelerations");
    }
    bool minimizeAuxiliaryDerivatives =
            m_solver.getMinimizeImplicitAuxiliaryDerivatives() &&
            m_problem.getNumAuxiliaryResidualEquations();
    if (minimizeAuxiliaryDerivatives) {
        m_objectiveTermNames.push_back("auxiliary_derivatives");
    }
    bool minimizeStateProjection =
            m_solver.getMinimizeStateProjection() &&
            m_numProjectionStates &&
            m_problem.isKinematicConstraintMethodBordalba2023();
    if (minimizeStateProjection) {
        m_objectiveTermNames.push_back("state_projection");
    }
    m_objectiveTerms = MX::zeros((int)m_objectiveTermNames.size(), 1);

    int iterm = 0;
    for (int ic = 0; ic < m_problem.getNumCosts(); ++ic) {
        const auto& info = m_problem.getCostInfos()[ic];

        MX integral;
        if (info.integrand_function) {
            // Here, we include evaluations of the integral cost
            // integrand into the symbolic expression graph for the integral
            // cost. We are *not* numerically evaluating the integral cost
            // integrand here--that occurs when the function by casadi::nlpsol()
            // is evaluated.
            MX integrandTraj = evalOnTrajectory(*info.integrand_function,
                    {states, controls, multipliers, derivatives}, m_gridIndices)
                    .at(0);

            integral = m_duration(0) * dot(quadCoeffs.T(), integrandTraj);
        } else {
            integral = MX::nan(1, 1);
        }

        MXVector costOut;
        info.endpoint_function->call(
                {m_unscaledVars[initial_time](0),
                        m_unscaledVars[states](Slice(), 0),
                        m_unscaledVars[controls](Slice(), 0),
                        m_unscaledVars[multipliers](Slice(), 0),
                        m_unscaledVars[derivatives](Slice(), 0),
                        m_unscaledVars[final_time](0),
                        m_unscaledVars[states](Slice(), -1),
                        m_unscaledVars[controls](Slice(), -1),
                        m_unscaledVars[multipliers](Slice(), -1),
                        m_unscaledVars[derivatives](Slice(), -1),
                        m_unscaledVars[parameters](Slice(), -1),
                        integral},
                costOut);
        m_objectiveTerms(iterm++) = casadi::MX::sum1(costOut.at(0));
    }

    // Minimize Lagrange multipliers if specified by the solver.
    if (minimizeLagrangeMultipliers) {
        const auto mults = m_scaledVars[multipliers];
        const double multiplierWeight = m_solver.getLagrangeMultiplierWeight();
        // Sum across constraints of each multiplier element squared.
        MX integrandTraj = MX::sum1(MX::sq(mults));
        m_objectiveTerms(iterm++) = multiplierWeight * m_duration(0) *
                       dot(quadCoeffs.T(), integrandTraj);
    }

    // Minimize generalized accelerations.
    if (minimizeAccelerations) {
        const auto& numAccels = m_problem.getNumAccelerations();
        const auto accels =
                m_scaledVars[derivatives](Slice(0, numAccels), Slice());
        const double accelWeight =
                m_solver.getImplicitMultibodyAccelerationsWeight();
        MX integrandTraj = MX::sum1(MX::sq(accels));
        m_objectiveTerms(iterm++) = accelWeight * m_duration(0) *
                       dot(quadCoeffs.T(), integrandTraj);
    }

    // Minimize auxiliary derivatives.
    if (minimizeAuxiliaryDerivatives) {
        const auto& numAccels = m_problem.getNumAccelerations();
        const auto& numAuxDerivs = m_problem.getNumAuxiliaryResidualEquations();
        const auto auxDerivs = m_scaledVars[derivatives](
                Slice(numAccels, numAccels + numAuxDerivs), Slice());
        const double auxDerivWeight =
                m_solver.getImplicitAuxiliaryDerivativesWeight();
        MX integrandTraj = MX::sum1(MX::sq(auxDerivs));
        m_objectiveTerms(iterm++) = auxDerivWeight * m_duration(0) *
                       dot(quadCoeffs.T(), integrandTraj);
    }

    // Minimize state projection distance.
    if (minimizeStateProjection && m_numProjectionStates) {
        for (int imesh = 0, istart = 0; imesh < m_numMeshIntervals; ++imesh) {
            casadi_int iend = istart + m_numPointsPerMeshInterval - 1;
            m_projectionStateDistances(Slice(), imesh) =
                m_unscaledVars[projection_states](Slice(), imesh) -
                m_unscaledVars[states](Slice(0, m_numProjectionStates), iend);
            istart = iend;
        }
        m_objectiveTerms(iterm++) = m_solver.getStateProjectionWeight() *
                       MX::sum2(MX::sum1(MX::sq(m_projectionStateDistances)));
    }

    // Endpoint constraints
    // --------------------
    int numEndpointConstraints =
            (int)m_problem.getEndpointConstraintInfos().size();
    m_constraints.endpoint.resize(numEndpointConstraints);
    m_constraintsLowerBounds.endpoint.resize(numEndpointConstraints);
    m_constraintsUpperBounds.endpoint.resize(numEndpointConstraints);
    for (int iec = 0; iec < (int)m_constraints.endpoint.size(); ++iec) {
        const auto& info = m_problem.getEndpointConstraintInfos()[iec];

        MX integral;
        if (info.integrand_function) {
            MX integrandTraj = evalOnTrajectory(*info.integrand_function,
                    {states, controls, multipliers, derivatives}, m_gridIndices)
                                       .at(0);

            integral = m_duration(0) * dot(quadCoeffs.T(), integrandTraj);
        } else {
            integral = MX::nan(1, 1);
        }

        MXVector endpointOut;
        info.endpoint_function->call(
                {m_unscaledVars[initial_time](-1),
                        m_unscaledVars[states](Slice(), 0),
                        m_unscaledVars[controls](Slice(), 0),
                        m_unscaledVars[multipliers](Slice(), 0),
                        m_unscaledVars[derivatives](Slice(), 0),
                        m_unscaledVars[final_time](-1),
                        m_unscaledVars[states](Slice(), -1),
                        m_unscaledVars[controls](Slice(), -1),
                        m_unscaledVars[multipliers](Slice(), -1),
                        m_unscaledVars[derivatives](Slice(), -1),
                        m_unscaledVars[parameters](Slice(), -1),
                        integral},
                endpointOut);
        m_constraints.endpoint[iec] = endpointOut.at(0);
        m_constraintsLowerBounds.endpoint[iec] = info.lowerBounds;
        m_constraintsUpperBounds.endpoint[iec] = info.upperBounds;
    }
}

Solution Transcription::solve(const Iterate& guessOrig) {

    // Define the NLP.
    // ---------------
    transcribe();

    // Resample the guess.
    // -------------------
    const auto guessTimes = createTimes(guessOrig.variables.at(initial_time),
            guessOrig.variables.at(final_time));
    bool appendProjectionStates =
            m_problem.getNumProjectionConstraintEquations();
    auto guess = guessOrig.resample(guessTimes, appendProjectionStates);
    guess = guess.repmatParameters(m_numMeshPoints);

    // Adjust guesses for the slack variables to ensure they are the correct
    // length (i.e. slacks.size2() == m_numPointsIgnoringConstraints).
    if (guess.variables.find(Var::slacks) != guess.variables.end()) {
        auto& slacks = guess.variables.at(Var::slacks);

        // If slack variables provided in the guess are equal to the grid
        // length, remove the elements on the mesh points where the slack
        // variables are not defined.
        if (slacks.size2() == m_numGridPoints) {
            casadi::DM meshIndices = createMeshIndices();
            std::vector<casadi_int> slackColumnsToRemove;
            if (m_problem.isKinematicConstraintMethodBordalba2023()) {
                slackColumnsToRemove.push_back(0);
            }
            for (int itime = 0; itime < m_numGridPoints; ++itime) {
                if (m_problem.isKinematicConstraintMethodBordalba2023()) {
                    if (!meshIndices(itime).__nonzero__()) {
                        slackColumnsToRemove.push_back(itime);
                    }
                } else {
                    if (meshIndices(itime).__nonzero__()) {
                        slackColumnsToRemove.push_back(itime);
                    }
                }
            }
            // The first argument is an empty vector since we don't want to
            // remove an entire row.
            slacks.remove(std::vector<casadi_int>(), slackColumnsToRemove);
        }

        // Check that either that the slack variables provided in the guess
        // are the correct length, or that the correct number of columns
        // were removed.
        int slackLength;
        if (m_problem.isKinematicConstraintMethodBordalba2023()) {
            slackLength = m_numMeshIntervals;
        } else {
            slackLength = m_numMeshInteriorPoints;
        }
        OPENSIM_THROW_IF(slacks.size2() != slackLength,
                OpenSim::Exception,
                "Expected slack variables to be length {}, but they are length "
                "{}.",
                slackLength, slacks.size2());
    }

    // Adjust guesses for the projection variables to ensure they are the
    // correct length.
    if (guess.variables.find(Var::projection_states) != guess.variables.end()) {
        auto& projection_states = guess.variables.at(Var::projection_states);

        if (projection_states.size2() == m_numGridPoints) {
            casadi::DM meshIndices = createMeshIndices();
            std::vector<casadi_int> columnsToRemove;
            columnsToRemove.push_back(0);

            for (int itime = 0; itime < m_numGridPoints; ++itime) {
                if (!meshIndices(itime).__nonzero__()) {
                    columnsToRemove.push_back(itime);
                }
            }
            // The first argument is an empty vector since we don't want to
            // remove an entire row.
            projection_states.remove(std::vector<casadi_int>(), columnsToRemove);
        }

        OPENSIM_THROW_IF(projection_states.size2() != m_numMeshIntervals,
                OpenSim::Exception,
                "Expected projection state variables to be length {}, but they "
                "are length {}.",
                m_numMeshIntervals, projection_states.size2());
    }

    // Create the CasADi NLP function.
    // -------------------------------
    // Option handling is copied from casadi::OptiNode::solver().
    casadi::Dict options = m_solver.getPluginOptions();
    if (!options.empty()) {
        options[m_solver.getOptimSolver()] = m_solver.getSolverOptions();
    }

    auto x = flattenVariables(m_scaledVectorVars);
    casadi_int numVariables = x.numel();

    // The m_constraints symbolic vector holds all of the expressions for
    // the constraint functions.
    auto g = flattenConstraints(m_constraints);
    casadi_int numConstraints = g.numel();

    NlpsolCallback callback(*this, m_problem, numVariables, numConstraints,
            m_solver.getCallbackInterval());
    options["iteration_callback"] = callback;

    // The inputs to nlpsol() are symbolic (casadi::MX).
    casadi::MXDict nlp;
    nlp.emplace(std::make_pair("x", x));
    // The objective symbolic variable holds an expression graph including
    // all the calculations performed on the variables x.
    casadi::MX objective = MX::sum1(m_objectiveTerms);
    if (m_objectiveTerms.numel() == 0) {
        objective = 0;
    }
    nlp.emplace(std::make_pair("f", objective));
    nlp.emplace(std::make_pair("g", g));
    if (!m_solver.getWriteSparsity().empty()) {
        const auto prefix = m_solver.getWriteSparsity();
        auto gradient = casadi::MX::gradient(nlp["f"], nlp["x"]);
        gradient.sparsity().to_file(
                prefix + "_objective_gradient_sparsity.mtx");
        auto hessian = casadi::MX::hessian(nlp["f"], nlp["x"]);
        hessian.sparsity().to_file(prefix + "_objective_Hessian_sparsity.mtx");
        auto lagrangian = objective +
                          casadi::MX::dot(casadi::MX::ones(nlp["g"].sparsity()),
                                  nlp["g"]);
        auto hessian_lagr = casadi::MX::hessian(lagrangian, nlp["x"]);
        hessian_lagr.sparsity().to_file(
                prefix + "_Lagrangian_Hessian_sparsity.mtx");
        auto jacobian = casadi::MX::jacobian(nlp["g"], nlp["x"]);
        jacobian.sparsity().to_file(
                prefix + "constraint_Jacobian_sparsity.mtx");
    }

    const auto lbg = flattenConstraints(m_constraintsLowerBounds);
    const auto ubg = flattenConstraints(m_constraintsUpperBounds);
    if (m_solver.getOptimSolver() == "fatrop") {
        options["structure_detection"] = "auto";
        std::vector<bool> equality;
        for (int i = 0; i < m_numConstraints; ++i) {
            equality.push_back(lbg(i).scalar() == ubg(i).scalar());
        }
        options["equality"] = equality;
        options["debug"] = true;
    }
    const casadi::Function nlpFunc =
            casadi::nlpsol("nlp", m_solver.getOptimSolver(), nlp, options);

    // Run the optimization (evaluate the CasADi NLP function).
    // --------------------------------------------------------
    // The inputs and outputs of nlpFunc are numeric (casadi::DM).
    const casadi::DMDict nlpResult = nlpFunc(casadi::DMDict{
                    {"x0", flattenVariables(scaleVariables(guess.variables))},
                    {"lbx", flattenVariables(scaleVariables(m_lowerBounds))},
                    {"ubx", flattenVariables(scaleVariables(m_upperBounds))},
                    {"lbg", lbg},
                    {"ubg", ubg}});

    // Create a CasOC::Solution.
    // -------------------------
    Solution solution = m_problem.createIterate<Solution>();
    const auto finalVariables = nlpResult.at("x");
    solution.variables = unscaleVariables(expandVariables(finalVariables));
    calcInterpolatingControls(solution.variables);
    solution.objective = nlpResult.at("f").scalar();

    casadi::DMVector finalVarsDMV{finalVariables};
    casadi::Function objectiveFunc("objective", {x}, {m_objectiveTerms});
    casadi::DMVector objectiveOut;
    objectiveFunc.call(finalVarsDMV, objectiveOut);
    solution.objective_breakdown = expandObjectiveTerms(objectiveOut[0]);

    solution.times = createTimes(
            solution.variables[initial_time], solution.variables[final_time]);
    solution.stats = nlpFunc.stats();

    // Print breakdown of objective.
    printObjectiveBreakdown(solution, objectiveOut[0]);

    if (!solution.stats.at("success")) {

        // For some reason, nlpResult.at("g") is all 0. So we calculate the
        // constraints ourselves.
        casadi::Function constraintFunc("constraints", {x}, {g});
        casadi::DMVector constraintsOut;
        constraintFunc.call(finalVarsDMV, constraintsOut);
        printConstraintValues(solution, expandConstraints(constraintsOut[0]));
    }
    return solution;
}

void Transcription::printConstraintValues(const Iterate& it,
        const Constraints<casadi::DM>& constraints,
        std::ostream& stream) const {

    std::stringstream ss;

    // Find the longest state, control, multiplier, derivative, or slack name.
    auto compareSize = [](const std::string& a, const std::string& b) {
        return a.size() < b.size();
    };
    int maxNameLength = 0;
    auto updateMaxNameLength = [&maxNameLength, compareSize](
                                       const std::vector<std::string>& names) {
        if (!names.empty()) {
            maxNameLength = (int)std::max_element(
                    names.begin(), names.end(), compareSize)
                                    ->size();
        }
    };
    updateMaxNameLength(it.state_names);
    updateMaxNameLength(it.control_names);
    updateMaxNameLength(it.multiplier_names);
    updateMaxNameLength(it.derivative_names);
    updateMaxNameLength(it.slack_names);

    ss << "\nActive or violated continuous variable bounds" << std::endl;
    ss << "L and U indicate which bound is active; "
              "'*' indicates a bound is violated. "
           << std::endl;
    ss << "The case of lower==upper==value is ignored." << std::endl;

    // Bounds on time-varying variables.
    // ---------------------------------
    auto print_bounds = [&ss, maxNameLength](const std::string& description,
                                const std::vector<std::string>& names,
                                const casadi::DM& times,
                                const casadi::DM& values,
                                const casadi::DM& lower,
                                const casadi::DM& upper) {
        ss << "\n" << description << ": ";

        bool boundsActive = false;
        bool boundsViolated = false;
        for (casadi_int ivar = 0; ivar < values.rows(); ++ivar) {
            for (casadi_int itime = 0; itime < times.numel(); ++itime) {
                const auto& L = lower(ivar, itime).scalar();
                const auto& V = values(ivar, itime).scalar();
                const auto& U = upper(ivar, itime).scalar();
                if (V <= L || V >= U) {
                    if (V == L && L == U) continue;
                    boundsActive = true;
                    if (V < L || V > U) {
                        boundsViolated = true;
                        break;
                    }
                }
            }
        }

        if (!boundsActive && !boundsViolated) {
            ss << "no bounds active or violated" << std::endl;
            return;
        }

        if (!boundsViolated) {
            ss << "some bounds active but no bounds violated";
        } else {
            ss << "some bounds active or violated";
        }

        ss << "\n"
               << std::setw(maxNameLength) << "  " << std::setw(9) << "time "
               << "  " << std::setw(9) << "lower"
               << "    " << std::setw(9) << "value"
               << "    " << std::setw(9) << "upper"
               << " " << std::endl;

        for (casadi_int ivar = 0; ivar < values.rows(); ++ivar) {
            for (casadi_int itime = 0; itime < times.numel(); ++itime) {
                const auto& L = lower(ivar, itime).scalar();
                const auto& V = values(ivar, itime).scalar();
                const auto& U = upper(ivar, itime).scalar();
                if (V <= L || V >= U) {
                    // In the case where lower==upper==value, there is no
                    // issue; ignore.
                    if (V == L && L == U) continue;
                    const auto& time = times(itime);
                    ss << std::setw(maxNameLength) << names[ivar] << "  "
                           << std::setprecision(2) << std::scientific
                           << std::setw(9) << time << "  " << std::setw(9) << L
                           << " <= " << std::setw(9) << V
                           << " <= " << std::setw(9) << U << " ";
                    // Show if the constraint is violated.
                    if (V <= L)
                        ss << "L";
                    else
                        ss << " ";
                    if (V >= U)
                        ss << "U";
                    else
                        ss << " ";
                    if (V < L || V > U) ss << "*";
                    ss << std::endl;
                }
            }
        }
    };
    const auto& vars = it.variables;
    const auto& lower = m_lowerBounds;
    const auto& upper = m_upperBounds;
    print_bounds("State bounds", it.state_names, it.times,
            vars.at(states), lower.at(states), upper.at(states));
    // TODO how to print bounds when controls are extrapolated?
    print_bounds("Control bounds", it.control_names, it.times,
            vars.at(controls), lower.at(controls), upper.at(controls));
    print_bounds("Multiplier bounds", it.multiplier_names, it.times,
            vars.at(multipliers), lower.at(multipliers), upper.at(multipliers));
    print_bounds("Derivative bounds", it.derivative_names, it.times,
            vars.at(derivatives), lower.at(derivatives), upper.at(derivatives));
    // Need to update times for the slacks:
    // print_bounds("Slack bounds", it.slack_names, it.times, vars.at(slacks),
    //         lower.at(slacks), upper.at(slacks));

    // Bounds on time and parameter variables.
    // ---------------------------------------
    maxNameLength = 0;
    updateMaxNameLength(it.parameter_names);
    std::vector<std::string> time_names = {"initial_time", "final_time"};
    updateMaxNameLength(time_names);

    ss << "\nActive or violated parameter bounds" << std::endl;
    ss << "L and U indicate which bound is active; "
              "'*' indicates a bound is violated. "
           << std::endl;
    ss << "The case of lower==upper==value is ignored." << std::endl;

    auto printParameterBounds = [&ss, maxNameLength](
                                        const std::string& description,
                                        const std::vector<std::string>& names,
                                        const casadi::DM& values,
                                        const casadi::DM& lower,
                                        const casadi::DM& upper) {
        ss << "\n" << description << ": ";

        bool boundsActive = false;
        bool boundsViolated = false;
        for (casadi_int ivar = 0; ivar < values.rows(); ++ivar) {
            const auto& L = lower(ivar).scalar();
            const auto& V = values(ivar).scalar();
            const auto& U = upper(ivar).scalar();
            if (V <= L || V >= U) {
                if (V == L && L == U) continue;
                boundsActive = true;
                if (V < L || V > U) {
                    boundsViolated = true;
                    break;
                }
            }
        }

        if (!boundsActive && !boundsViolated) {
            ss << "no bounds active or violated" << std::endl;
            return;
        }

        if (!boundsViolated) {
            ss << "some bounds active but no bounds violated";
        } else {
            ss << "some bounds active or violated";
        }

        ss << "\n"
               << std::setw(maxNameLength) << "  " << std::setw(9) << "lower"
               << "    " << std::setw(9) << "value"
               << "    " << std::setw(9) << "upper"
               << " " << std::endl;

        for (casadi_int ivar = 0; ivar < values.rows(); ++ivar) {
            const auto& L = lower(ivar).scalar();
            const auto& V = values(ivar).scalar();
            const auto& U = upper(ivar).scalar();
            if (V <= L || V >= U) {
                // In the case where lower==upper==value, there is no
                // issue; ignore.
                if (V == L && L == U) continue;
                ss << std::setw(maxNameLength) << names[ivar] << "  "
                       << std::setprecision(2) << std::scientific
                       << std::setw(9) << L << " <= " << std::setw(9) << V
                       << " <= " << std::setw(9) << U << " ";
                // Show if the constraint is violated.
                if (V <= L)
                    ss << "L";
                else
                    ss << " ";
                if (V >= U)
                    ss << "U";
                else
                    ss << " ";
                if (V < L || V > U) ss << "*";
                ss << std::endl;
            }
        }
    };
    casadi::DM timeValues(2, 1);
    timeValues(0) = vars.at(initial_time)(0);
    timeValues(1) = vars.at(final_time)(0);

    casadi::DM timeLower(2, 1);
    timeLower(0) = lower.at(initial_time)(0);
    timeLower(1) = lower.at(final_time)(0);

    casadi::DM timeUpper(2, 1);
    timeUpper(0) = upper.at(initial_time)(0);
    timeUpper(1) = upper.at(final_time)(0);

    printParameterBounds(
            "Time bounds", time_names, timeValues, timeLower, timeUpper);
    printParameterBounds("Parameter bounds", it.parameter_names,
            vars.at(parameters)(0), lower.at(parameters)(0),
            upper.at(parameters)(0));

    // Constraints.
    // ============
    ss << "\nTotal number of constraints: " << m_numConstraints << "."
           << std::endl;

    // Differential equation defects.
    // ------------------------------
    ss << "\nDifferential equation defects:"
           << "\n  L2 norm across mesh, max abs value (L1 norm), time of max "
              "abs"
           << std::endl;

    auto calcL1Norm = [](const casadi::DM& v, int& argmax) {
        double max = v(0).scalar();
        argmax = 0;
        for (int i = 1; i < v.numel(); ++i) {
            if (v(i).scalar() > max) {
                max = std::abs(v(i).scalar());
                argmax = i;
            }
        }
        return max;
    };

    std::string spacer(7, ' ');
    casadi::DM row(1, constraints.defects.columns());
    for (size_t istate = 0; istate < it.state_names.size(); ++istate) {
        row = constraints.defects(istate, Slice());
        const double L2 = casadi::DM::norm_2(row).scalar();
        int argmax;
        double max = calcL1Norm(row, argmax);
        const double L1 = max;
        const double time_of_max = it.times(argmax).scalar();

        ss << std::setw(maxNameLength) << it.state_names[istate] << spacer
               << std::setprecision(2) << std::scientific << std::setw(9) << L2
               << spacer << L1 << spacer << std::setprecision(6) << std::fixed
               << time_of_max << std::endl;
    }

    // Kinematic constraints.
    // ----------------------
    ss << "\nKinematic constraints:";
    std::vector<std::string> kinconNames =
            m_problem.createKinematicConstraintEquationNames();
    int numQErr = m_problem.getNumQErr();
    int numUErr = m_problem.getNumUErr();
    int numUDotErr = m_problem.getNumUDotErr();
    int numKC = m_problem.isKinematicConstraintMethodBordalba2023() ?
            numQErr + numUErr : numQErr + numUErr + numUDotErr;
    if (kinconNames.empty()) {
        ss << " none" << std::endl;
    } else {
        maxNameLength = 0;
        updateMaxNameLength(kinconNames);
        ss << "\n  L2 norm across mesh, max abs value (L1 norm), time of "
                    "max "
                    "abs"
                << std::endl;
        row.resize(1, m_numMeshPoints);
        {
            int ikc = 0;
            for (int i = 0; i < numKC; ++i) {
                row = constraints.kinematic(i, Slice());
                const double L2 = casadi::DM::norm_2(row).scalar();
                int argmax;
                double max = calcL1Norm(row, argmax);
                const double L1 = max;
                const double time_of_max = it.times(argmax).scalar();

                std::string label = kinconNames.at(ikc);
                ss << std::setfill('0') << std::setw(2) << ikc << ":"
                   << std::setfill(' ') << std::setw(maxNameLength) << label
                   << spacer << std::setprecision(2) << std::scientific
                   << std::setw(9) << L2 << spacer << L1 << spacer
                   << std::setprecision(6) << std::fixed << time_of_max
                   << std::endl;
                ++ikc;
            }
            if (m_problem.isKinematicConstraintMethodBordalba2023()) {
                for (int iudot = 0; iudot < numUDotErr; ++iudot) {
                    row = constraints.kinematic_udoterr(iudot, Slice());
                    const double L2 = casadi::DM::norm_2(row).scalar();
                    int argmax;
                    double max = calcL1Norm(row, argmax);
                    const double L1 = max;
                    const double time_of_max = it.times(argmax).scalar();

                    std::string label = kinconNames.at(ikc);
                    ss << std::setfill('0') << std::setw(2) << ikc << ":"
                    << std::setfill(' ') << std::setw(maxNameLength) << label
                    << spacer << std::setprecision(2) << std::scientific
                    << std::setw(9) << L2 << spacer << L1 << spacer
                    << std::setprecision(6) << std::fixed << time_of_max
                    << std::endl;
                    ++ikc;
                }
            }
        }
        ss << "Kinematic constraint values at each mesh point:"
                << std::endl;
        ss << "      time  ";
        for (int ipc = 0; ipc < (int)kinconNames.size(); ++ipc) {
            ss << std::setw(9) << ipc << "  ";
        }
        ss << std::endl;
        for (int imesh = 0; imesh < m_numMeshPoints; ++imesh) {
            ss << std::setfill('0') << std::setw(3) << imesh << "  ";
            ss.fill(' ');
            ss << std::setw(9) << it.times(imesh).scalar() << "  ";
            for (int i = 0; i < numKC; ++i) {
                const auto& value =
                        constraints.kinematic(i, imesh).scalar();
                ss << std::setprecision(2) << std::scientific
                   << std::setw(9) << value << "  ";
            }
            if (m_problem.isKinematicConstraintMethodBordalba2023()) {
                for (int iudot = 0; iudot < numUDotErr; ++iudot) {
                    const auto& value =
                            constraints.kinematic_udoterr(iudot, imesh).scalar();
                    ss << std::setprecision(2) << std::scientific
                    << std::setw(9) << value << "  ";
                }
            }
            ss << std::endl;
        }
    }

    // Path constraints.
    // -----------------
    ss << "\nPath constraints:";
    std::vector<std::string> pathconNames;
    for (const auto& pc : m_problem.getPathConstraintInfos()) {
        pathconNames.push_back(pc.name);
    }

    if (pathconNames.empty()) {
        ss << " none" << std::endl;
    } else {
        ss << std::endl;

        maxNameLength = 0;
        updateMaxNameLength(pathconNames);
        // To make space for indices.
        maxNameLength += 3;
        ss << "  L2 norm across mesh, max abs value (L1 norm), time of "
                  "max abs"
               << std::endl;
        row.resize(1, m_numMeshPoints);
        {
            int ipc = 0;
            for (const auto& pc : m_problem.getPathConstraintInfos()) {
                for (int ieq = 0; ieq < pc.size(); ++ieq) {
                    row = constraints.path[ipc](ieq, Slice());
                    const double L2 = casadi::DM::norm_2(row).scalar();
                    int argmax;
                    double max = calcL1Norm(row, argmax);
                    const double L1 = max;
                    const double time_of_max = it.times(argmax).scalar();

                    std::string label = fmt::format("{}_{}", pc.name, ieq);
                    ss << std::setfill('0') << std::setw(2) << ipc << ":"
                           << std::setfill(' ') << std::setw(maxNameLength)
                           << label << spacer << std::setprecision(2)
                           << std::scientific << std::setw(9) << L2 << spacer
                           << L1 << spacer << std::setprecision(6) << std::fixed
                           << time_of_max << std::endl;
                }
                ++ipc;
            }
        }
        ss << "\nPath constraint values at each path constraint point:" << std::endl;
        ss << "      time  ";
        for (int ipc = 0; ipc < (int)pathconNames.size(); ++ipc) {
            ss << std::setw(9) << ipc << "  ";
        }
        ss << std::endl;
        for (int ipcp = 0; ipcp < m_numPathConstraintPoints; ++ipcp) {
            ss << std::setfill('0') << std::setw(3) << ipcp << "  ";
            ss.fill(' ');
            ss << std::setw(9) << it.times(ipcp).scalar() << "  ";
            for (int ipc = 0; ipc < (int)pathconNames.size(); ++ipc) {
                const auto& value = constraints.path[ipc](ipcp).scalar();
                ss << std::setprecision(2) << std::scientific
                       << std::setw(9) << value << "  ";
            }
            ss << std::endl;
        }
    }
    if (stream.rdbuf() == std::cout.rdbuf()) {
        // TODO log_cout() does not work for Level::Warn and up.
        // OpenSim::log_cout(ss.str());
        std::cout << ss.str() << std::endl;
    } else {
        stream << ss.str() << std::endl;
    }
}

void Transcription::printObjectiveBreakdown(const Iterate& it,
        const casadi::DM& objectiveTerms,
        std::ostream& stream) const {
    std::stringstream ss;
    ss << "\nBreakdown of objective (including weights):";
    if (objectiveTerms.numel() == 0) {
        ss << " no terms";
    } else {
        for (int io = 0; io < (int)m_objectiveTermNames.size(); ++io) {
            ss << "\n  " << m_objectiveTermNames[io] << ": "
                   << objectiveTerms(io).scalar();
        }
    }
    if (stream.rdbuf() == std::cout.rdbuf()) {
        // TODO log_cout() does not work for Level::Warn and up.
        // OpenSim::log_cout(ss.str());
        std::cout << ss.str() << std::endl;
    } else {
        stream << ss.str() << std::endl;
    }
}

Iterate Transcription::createInitialGuessFromBounds() const {
    auto setToMidpoint = [](DM& output, const DM& lowerDM, const DM& upperDM) {
        for (int irow = 0; irow < output.rows(); ++irow) {
            for (int icol = 0; icol < output.columns(); ++icol) {
                const auto& lower = double(lowerDM(irow, icol));
                const auto& upper = double(upperDM(irow, icol));
                if (!std::isinf(lower) && !std::isinf(upper)) {
                    output(irow, icol) = 0.5 * (upper + lower);
                } else if (!std::isinf(lower))
                    output(irow, icol) = lower;
                else if (!std::isinf(upper))
                    output(irow, icol) = upper;
                else
                    output(irow, icol) = 0;
            }
        }
    };
    Iterate casGuess = m_problem.createIterate();
    casGuess.variables = m_lowerBounds;
    for (auto& kv : casGuess.variables) {
        setToMidpoint(kv.second, m_lowerBounds.at(kv.first),
                m_upperBounds.at(kv.first));
    }
    casGuess.times = createTimes(
            casGuess.variables[initial_time], casGuess.variables[final_time]);
    return casGuess;
}

Iterate Transcription::createRandomIterateWithinBounds(
        const SimTK::Random* randGen) const {
    static const SimTK::Random::Uniform randGenDefault(-1, 1);
    const SimTK::Random* randGenToUse = &randGenDefault;
    if (randGen) randGenToUse = randGen;
    auto setRandom = [&](DM& output, const DM& lowerDM, const DM& upperDM) {
        for (int irow = 0; irow < output.rows(); ++irow) {
            for (int icol = 0; icol < output.columns(); ++icol) {
                const auto& lower = double(lowerDM(irow, icol));
                const auto& upper = double(upperDM(irow, icol));
                const auto rand = randGenToUse->getValue();
                auto value = 0.5 * (rand + 1.0) * (upper - lower) + lower;
                if (std::isnan(value)) value = SimTK::clamp(lower, rand, upper);
                output(irow, icol) = value;
            }
        }
    };
    Iterate casIterate = m_problem.createIterate();
    casIterate.variables = m_lowerBounds;
    for (auto& kv : casIterate.variables) {
        setRandom(kv.second, m_lowerBounds.at(kv.first),
                m_upperBounds.at(kv.first));
    }
    casIterate.times = createTimes(casIterate.variables[initial_time],
            casIterate.variables[final_time]);
    return casIterate;
}

casadi::MXVector Transcription::evalOnTrajectory(
        const casadi::Function& pointFunction, const std::vector<Var>& inputs,
        const casadi::Matrix<casadi_int>& timeIndices) const {
    auto parallelism = m_solver.getParallelism();
    const auto trajFunc = pointFunction.map(
            timeIndices.size2(), parallelism.first, parallelism.second);

    // Assemble input.
    // Add 1 for time input and 1 for parameters input.
    MXVector mxIn(inputs.size() + 2);
    mxIn[0] = m_times(timeIndices);
    const auto NQ = m_problem.getNumCoordinates();
    const auto NU = m_problem.getNumSpeeds();
    const auto NS = m_problem.getNumStates();
    for (int i = 0; i < (int)inputs.size(); ++i) {
        if (inputs[i] == multibody_states) {
            mxIn[i + 1] =
                    m_unscaledVars.at(states)(Slice(0, NQ + NU), timeIndices);
        } else if (inputs[i] == projection_states) {
            const auto& proj_states = m_unscaledVars.at(projection_states);
            OPENSIM_ASSERT(proj_states.size2() == timeIndices.size2());
            mxIn[i + 1] = casadi::MX(NS, timeIndices.size2());
            mxIn[i + 1](Slice(0, NQ + NU), Slice()) =
                    m_unscaledVars.at(projection_states)(Slice(0, NQ + NU),
                            Slice());
            mxIn[i + 1](Slice(NQ + NU, NS), Slice()) =
                    m_unscaledVars.at(states)(Slice(NQ + NU, NS), timeIndices);
        } else if (inputs[i] == slacks) {
            mxIn[i + 1] = m_unscaledVars.at(inputs[i]);
        } else {
            mxIn[i + 1] = m_unscaledVars.at(inputs[i])(Slice(), timeIndices);
        }
    }
    mxIn[mxIn.size() - 1] = m_parameters(Slice(), timeIndices);

    MXVector mxOut;
    trajFunc.call(mxIn, mxOut);
    return mxOut;
    // TODO: Avoid the overhead of map() if not running in parallel.
    /* } else {
    casadi::MXVector out(pointFunction.n_out());
    for (int iout = 0; iout < (int)out.size(); ++iout) {
    out[iout] = casadi::MX(pointFunction.sparsity_out(iout).rows(),
    timeIndices.size2());
    }
    for (int itime = 0; itime < timeIndices.size2(); ++itime) {

    }
    }*/
}

} // namespace CasOC
