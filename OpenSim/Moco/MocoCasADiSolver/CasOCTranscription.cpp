/* -------------------------------------------------------------------------- *
 * OpenSim Moco: CasOCTranscription.cpp                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2018 Stanford University and the Authors                     *
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
#include "CasOCTranscription.h"

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
            iterate.variables = m_transcription.expandVariables(args.at(0));
            iterate.times =
                    m_transcription.createTimes(iterate.variables[initial_time],
                            iterate.variables[final_time]);
            iterate.iteration = evalCount;
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
        int numPointsPerMeshInterval,
        const casadi::DM& pointsForInterpControls) {
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
    m_pointsForInterpControls = pointsForInterpControls;
    m_numMultibodyResiduals = m_problem.isDynamicsModeImplicit()
                             ? m_problem.getNumMultibodyDynamicsEquations()
                             : 0;
    m_numAuxiliaryResiduals = m_problem.getNumAuxiliaryResidualEquations();
    const int numMultibodyStates =
            m_problem.getNumCoordinates() + m_problem.getNumSpeeds();
    m_numProjectionStates =
            m_problem.getNumProjectionConstraintEquations() &&
            m_problem.isKinematicConstraintMethodBordalba2023()
            ? numMultibodyStates : 0;

    m_numUDotErrorPoints = m_problem.isKinematicConstraintMethodBordalba2023()
            ? m_numGridPoints
            : m_numMeshPoints;
    m_numConstraints =
            m_numDefectsPerMeshInterval * m_numMeshIntervals +
            m_numMultibodyResiduals * m_numGridPoints +
            m_numAuxiliaryResiduals * m_numGridPoints +
            m_problem.getNumQErr() * m_numMeshPoints +
            m_problem.getNumUErr() * m_numMeshPoints +
            m_problem.getNumUDotErr() * m_numUDotErrorPoints +
            m_problem.getNumControls() * (int)pointsForInterpControls.numel() +
            m_problem.getNumProjectionConstraintEquations() * m_numMeshIntervals;
    m_constraints.endpoint.resize(
            m_problem.getEndpointConstraintInfos().size());
    for (int iec = 0; iec < (int)m_constraints.endpoint.size(); ++iec) {
        const auto& info = m_problem.getEndpointConstraintInfos()[iec];
        m_numConstraints += info.num_outputs;
    }
    m_constraints.path.resize(m_problem.getPathConstraintInfos().size());
    m_numPathConstraintPoints =
            m_solver.getEnforcePathConstraintMeshInteriorPoints()
            ? m_numGridPoints : m_numMeshPoints;
    for (int ipc = 0; ipc < (int)m_constraints.path.size(); ++ipc) {
        const auto& info = m_problem.getPathConstraintInfos()[ipc];
        m_numConstraints += info.size() * m_numPathConstraintPoints;
    }
    m_grid = grid;

    // Create variables.
    // -----------------
    m_scaledVars[initial_time] = MX::sym("initial_time");
    m_scaledVars[final_time] = MX::sym("final_time");
    m_scaledVars[states] =
            MX::sym("states", m_problem.getNumStates(), m_numGridPoints);
    m_scaledVars[projection_states] = MX::sym(
            "projection_states", m_numProjectionStates, m_numMeshIntervals);
    m_scaledVars[controls] =
            MX::sym("controls", m_problem.getNumControls(), m_numGridPoints);
    m_scaledVars[multipliers] = MX::sym(
            "multipliers", m_problem.getNumMultipliers(), m_numGridPoints);
    m_scaledVars[derivatives] = MX::sym(
            "derivatives", m_problem.getNumDerivatives(), m_numGridPoints);
    m_scaledVars[parameters] =
            MX::sym("parameters", m_problem.getNumParameters(), 1);

    if (m_problem.isKinematicConstraintMethodBordalba2023()) {
        // In the projection method for enforcing kinematic constraints, the
        // slack variables are applied at the mesh points at the end of each
        // mesh interval.
        m_scaledVars[slacks] = MX::sym(
                "slacks", m_problem.getNumSlacks(), m_numMeshIntervals);
    } else {
        // In the Posa et al. 2016 method for enforcing kinematic constraints,
        // the mesh interior points will be the mesh interval midpoints in the
        // Hermite-Simpson collocation scheme.
        m_scaledVars[slacks] = MX::sym(
                "slacks", m_problem.getNumSlacks(), m_numMeshInteriorPoints);
    }

    // Each element in the "defect states" vector is a matrix (i.e., MX)
    // containing the states needed to construct the defect constraints for an
    // individual mesh interval.
    m_statesByMeshInterval = MXVector(m_numMeshIntervals);
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        m_statesByMeshInterval[imesh] = MX(m_problem.getNumStates(),
                                   m_numPointsPerMeshInterval);
    }
    m_projectionStateDistances = MX(m_numProjectionStates, m_numMeshIntervals);

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
    m_projectionStateIndices = makeTimeIndices(projectionStateIndicesVector);
    m_notProjectionStateIndices =
            makeTimeIndices(notProjectionStateIndicesVector);

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

    setVariableBounds(initial_time, 0, 0, m_problem.getTimeInitialBounds());
    setVariableBounds(final_time, 0, 0, m_problem.getTimeFinalBounds());

    setVariableScaling(initial_time, 0, 0, m_problem.getTimeInitialBounds());
    setVariableScaling(final_time, 0, 0, m_problem.getTimeFinalBounds());

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
            setVariableScaling(states, Slice(), Slice(), info.bounds);
            ++is;
        }
    }
    {
        const auto& controlInfos = m_problem.getControlInfos();
        int ic = 0;
        for (const auto& info : controlInfos) {
            setVariableBounds(
                    controls, ic, Slice(1, m_numGridPoints - 1), info.bounds);
            setVariableBounds(controls, ic, 0, info.initialBounds);
            setVariableBounds(controls, ic, -1, info.finalBounds);
            setVariableScaling(controls, Slice(), Slice(), info.bounds);
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
            setVariableScaling(multipliers, Slice(), Slice(), info.bounds);
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
        const auto& stateInfos = m_problem.getStateInfos();
        for (int ias = 0; ias < m_numProjectionStates; ++ias) {
            const auto& info = stateInfos[ias];
            setVariableBounds(
                    projection_states, ias, Slice(0, m_numMeshIntervals - 1),
                    info.bounds);
            setVariableBounds(projection_states, ias, -1, info.finalBounds);
            setVariableScaling(projection_states, Slice(), Slice(),
                               info.bounds);
        }
    }
    {
        const auto& paramInfos = m_problem.getParameterInfos();
        int ip = 0;
        for (const auto& info : paramInfos) {
            setVariableBounds(parameters, ip, 0, info.bounds);
            setVariableScaling(parameters, ip, 0, info.bounds);
            ++ip;
        }
    }

    m_unscaledVars = unscaleVariables(m_scaledVars);

    m_duration = m_unscaledVars[final_time] - m_unscaledVars[initial_time];
    m_times = createTimes(
            m_unscaledVars[initial_time], m_unscaledVars[final_time]);
    m_paramsTrajGrid =
            MX::repmat(m_unscaledVars[parameters], 1, m_numGridPoints);
    m_paramsTrajMesh =
            MX::repmat(m_unscaledVars[parameters], 1, m_numMeshPoints);
    m_paramsTrajMeshInterior =
            MX::repmat(m_unscaledVars[parameters], 1, m_numMeshInteriorPoints);
    m_paramsTrajPathCon =
            MX::repmat(m_unscaledVars[parameters], 1, m_numPathConstraintPoints);
    m_paramsTrajProjState =
            MX::repmat(m_unscaledVars[parameters], 1, m_numMeshIntervals);

    casadi_int istart = 0;
    int numStates = m_problem.getNumStates();
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        casadi_int numPts = m_numPointsPerMeshInterval;
        casadi_int iend = istart + numPts - 1;
        if (m_numProjectionStates) {
            // The states at all points in the mesh interval except the last
            // point are the regular state variables.
            m_statesByMeshInterval[imesh](Slice(), Slice(0, numPts-1)) =
                    m_unscaledVars[states](Slice(), Slice(istart, iend));

            // The multibody states at the last point in the mesh interval are
            // the projection states.
            m_statesByMeshInterval[imesh](Slice(0, m_numProjectionStates), numPts-1) =
                    m_unscaledVars[projection_states](Slice(), imesh);

            // The non-multibody states at the last point (i.e., auxiliary state
            // variables for muscles) are also the same as the regular state
            // variables (there are no projection states for these variables).
            m_statesByMeshInterval[imesh](
                    Slice(m_numProjectionStates, numStates), numPts-1) =
                    m_unscaledVars[states](
                            Slice(m_numProjectionStates, numStates), iend);

            // Calculate the distance between the regular multibody states and
            // the projection multibody states.
            m_projectionStateDistances(Slice(), imesh) =
                    m_unscaledVars[projection_states](Slice(), imesh) -
                    m_unscaledVars[states](Slice(0, m_numProjectionStates), iend);
        } else {
            m_statesByMeshInterval[imesh](Slice(), Slice()) =
                    m_unscaledVars[states](Slice(), Slice(istart, iend+1));
        }
        istart = iend;
    }
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

    // TODO: Does creating all this memory have efficiency implications
    //        in CasADi?
    // Initialize memory for state derivatives and defects.
    // ----------------------------------------------------
    m_xdot = MX(NS, m_numGridPoints);
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
    const auto& kcBounds = m_problem.getKinematicConstraintBounds();

    m_constraints.kinematic_qerr = MX(
            casadi::Sparsity::dense(m_problem.getNumQErr(), m_numMeshPoints));
    m_constraintsLowerBounds.kinematic_qerr = casadi::DM::repmat(
            kcBounds.lower, m_problem.getNumQErr(), m_numMeshPoints);
    m_constraintsUpperBounds.kinematic_qerr = casadi::DM::repmat(
            kcBounds.upper, m_problem.getNumQErr(), m_numMeshPoints);

    m_constraints.kinematic_uerr = MX(
            casadi::Sparsity::dense(m_problem.getNumUErr(), m_numMeshPoints));
    m_constraintsLowerBounds.kinematic_uerr = casadi::DM::repmat(
            kcBounds.lower, m_problem.getNumUErr(), m_numMeshPoints);
    m_constraintsUpperBounds.kinematic_uerr = casadi::DM::repmat(
            kcBounds.upper, m_problem.getNumUErr(), m_numMeshPoints);

    m_constraints.kinematic_udoterr = MX(
            casadi::Sparsity::dense(
                    m_problem.getNumUDotErr(), m_numUDotErrorPoints));
    m_constraintsLowerBounds.kinematic_udoterr = casadi::DM::repmat(
            kcBounds.lower, m_problem.getNumUDotErr(), m_numUDotErrorPoints);
    m_constraintsUpperBounds.kinematic_udoterr = casadi::DM::repmat(
            kcBounds.upper, m_problem.getNumUDotErr(), m_numUDotErrorPoints);

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

    if (m_problem.getNumKinematicConstraintEquations() &&
            !m_problem.isPrescribedKinematics() &&
            m_problem.getEnforceConstraintDerivatives()) {

        OPENSIM_THROW_IF(!m_problem.isKinematicConstraintMethodBordalba2023() &&
                         m_solver.getTranscriptionScheme() != "hermite-simpson",
            OpenSim::Exception,
            "Expected the 'hermite-simpson' transcription scheme when using "
            "the Posa et al. 2016 method for enforcing kinematic constraints, "
            "but the '{}' "
            "scheme was selected.", m_solver.getTranscriptionScheme());

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

        if (m_problem.isKinematicConstraintMethodBordalba2023()) {
            const auto projectionOut = evalOnTrajectory(
                    m_problem.getStateProjection(),
                    {multibody_states, slacks},
                    m_projectionStateIndices);
            const auto x_proj = projectionOut.at(0);

            m_constraints.projection =
                    m_unscaledVars[states](Slice(0, NQ + NU),
                            m_projectionStateIndices) -
                    m_unscaledVars[projection_states] -
                    x_proj;
        }
    }

    // udot, zdot, residual, kcerr
    // ---------------------------
    if (m_problem.isDynamicsModeImplicit()) {
        // udot.
        const MX w = m_unscaledVars[derivatives](Slice(0, m_problem.getNumSpeeds()),
                Slice());
        m_xdot(Slice(NQ, NQ + NU), Slice()) = w;

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
            m_constraints.kinematic_qerr = out.at(3);
            m_constraints.kinematic_uerr = out.at(4);
            m_constraints.kinematic_udoterr = out.at(5);
//            if (m_problem.isKinematicConstraintMethodBordalba2023()) {
//                m_constraints.kinematic_udoterr(Slice(), m_meshIndices) =
//                        out.at(5);
//            } else {
//                m_constraints.kinematic_udoterr = out.at(5);
//            }
        }

        // Points where we ignore algebraic constraints.
        if (m_numMeshInteriorPoints) {
//            const casadi::Function& implicitMultibodyFunction =
//                m_problem.isKinematicConstraintMethodBordalba2023()
//                    ? m_problem.getImplicitMultibodySystemAccelerationConstraints()
//                    : m_problem.getImplicitMultibodySystemIgnoringConstraints();
            const auto out = evalOnTrajectory(
                    m_problem.getImplicitMultibodySystemIgnoringConstraints(),
                    inputs, m_meshInteriorIndices);
            m_constraints.multibody_residuals(Slice(), m_meshInteriorIndices) =
                    out.at(0);
            // zdot.
            m_xdot(Slice(NQ + NU, NS), m_meshInteriorIndices) =
                    out.at(1);
            m_constraints.auxiliary_residuals(Slice(), m_meshInteriorIndices) =
                    out.at(2);
//            if (m_problem.isKinematicConstraintMethodBordalba2023()) {
//                m_constraints.kinematic_udoterr(Slice(), m_meshInteriorIndices) =
//                        out.at(5);
//            }
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
            m_constraints.kinematic_qerr = out.at(3);
            m_constraints.kinematic_uerr = out.at(4);
            m_constraints.kinematic_udoterr = out.at(5);
//            if (m_problem.isKinematicConstraintMethodBordalba2023()) {
//                m_constraints.kinematic_udoterr(Slice(), m_meshIndices) =
//                        out.at(5);
//            } else {
//                m_constraints.kinematic_udoterr = out.at(5);
//            }
        }

        // Points where we ignore algebraic constraints.
        if (m_numMeshInteriorPoints) {
//            const casadi::Function& multibodyFunction =
//                m_problem.isKinematicConstraintMethodBordalba2023()
//                    ? m_problem.getMultibodySystemAccelerationConstraints()
//                    : m_problem.getMultibodySystemIgnoringConstraints();

            const auto out = evalOnTrajectory(
                    m_problem.getMultibodySystemIgnoringConstraints(), inputs,
                    m_meshInteriorIndices);
            m_xdot(Slice(NQ, NQ + NU), m_meshInteriorIndices) =
                    out.at(0);
            m_xdot(Slice(NQ + NU, NS), m_meshInteriorIndices) =
                    out.at(1);
            m_constraints.auxiliary_residuals(Slice(), m_meshInteriorIndices) =
                    out.at(2);
//            if (m_problem.isKinematicConstraintMethodBordalba2023()) {
//                m_constraints.kinematic_udoterr(Slice(), m_meshInteriorIndices) =
//                        out.at(5);
//            }
        }
    }

    // Calculate defects.
    // ------------------
    calcDefects();

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

    // Interpolating controls.
    // -----------------------
    m_constraints.interp_controls =
            casadi::DM(casadi::Sparsity::dense(m_problem.getNumControls(),
                    (int)m_pointsForInterpControls.numel()));
    const auto boundsOnInterpControls = casadi::DM::zeros(
            m_problem.getNumControls(), (int)m_pointsForInterpControls.numel());
    m_constraintsLowerBounds.interp_controls = boundsOnInterpControls;
    m_constraintsUpperBounds.interp_controls = boundsOnInterpControls;

    calcInterpolatingControls();
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
            m_problem.getNumKinematicConstraintEquations() &&
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

            integral = m_duration * dot(quadCoeffs.T(), integrandTraj);
        } else {
            integral = MX::nan(1, 1);
        }

        MXVector costOut;
        info.endpoint_function->call(
                {m_unscaledVars[initial_time],
                        m_unscaledVars[states](Slice(), 0),
                        m_unscaledVars[controls](Slice(), 0),
                        m_unscaledVars[multipliers](Slice(), 0),
                        m_unscaledVars[derivatives](Slice(), 0),
                        m_unscaledVars[final_time],
                        m_unscaledVars[states](Slice(), -1),
                        m_unscaledVars[controls](Slice(), -1),
                        m_unscaledVars[multipliers](Slice(), -1),
                        m_unscaledVars[derivatives](Slice(), -1),
                        m_unscaledVars[parameters], 
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
        m_objectiveTerms(iterm++) = multiplierWeight * m_duration *
                       dot(quadCoeffs.T(), integrandTraj);
    }

    // Minimize generalized accelerations.
    if (minimizeAccelerations) {
        const auto& numAccels = m_problem.getNumAccelerations();
        const auto accels = m_scaledVars[derivatives](
                Slice(0, numAccels), Slice());
        const double accelWeight =
                m_solver.getImplicitMultibodyAccelerationsWeight();
        MX integrandTraj = MX::sum1(MX::sq(accels));
        m_objectiveTerms(iterm++) = accelWeight * m_duration *
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
        m_objectiveTerms(iterm++) = auxDerivWeight * m_duration *
                       dot(quadCoeffs.T(), integrandTraj);
    }

    // Minimize state projection distance.
    if (minimizeStateProjection && m_numProjectionStates) {
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

            integral = m_duration * dot(quadCoeffs.T(), integrandTraj);
        } else {
            integral = MX::nan(1, 1);
        }

        MXVector endpointOut;
        info.endpoint_function->call(
                {m_unscaledVars[initial_time],
                        m_unscaledVars[states](Slice(), 0),
                        m_unscaledVars[controls](Slice(), 0),
                        m_unscaledVars[multipliers](Slice(), 0),
                        m_unscaledVars[derivatives](Slice(), 0),
                        m_unscaledVars[final_time],
                        m_unscaledVars[states](Slice(), -1),
                        m_unscaledVars[controls](Slice(), -1),
                        m_unscaledVars[multipliers](Slice(), -1),
                        m_unscaledVars[derivatives](Slice(), -1),
                        m_unscaledVars[parameters],
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
            m_problem.getNumKinematicConstraintEquations() &&
            m_problem.isKinematicConstraintMethodBordalba2023();
    auto guess = guessOrig.resample(guessTimes, appendProjectionStates);

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

    auto x = flattenVariables(m_scaledVars);
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
    const casadi::Function nlpFunc =
            casadi::nlpsol("nlp", m_solver.getOptimSolver(), nlp, options);

    // Run the optimization (evaluate the CasADi NLP function).
    // --------------------------------------------------------
    // The inputs and outputs of nlpFunc are numeric (casadi::DM).
    const casadi::DMDict nlpResult = nlpFunc(casadi::DMDict{
                    {"x0", flattenVariables(scaleVariables(guess.variables))},
                    {"lbx", flattenVariables(scaleVariables(m_lowerBounds))},
                    {"ubx", flattenVariables(scaleVariables(m_upperBounds))},
                    {"lbg", flattenConstraints(m_constraintsLowerBounds)},
                    {"ubg", flattenConstraints(m_constraintsUpperBounds)}});

    // Create a CasOC::Solution.
    // -------------------------
    Solution solution = m_problem.createIterate<Solution>();
    const auto finalVariables = nlpResult.at("x");
    solution.variables = unscaleVariables(expandVariables(finalVariables));
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
    print_bounds("State bounds", it.state_names, it.times, vars.at(states),
            lower.at(states), upper.at(states));
    print_bounds("Control bounds", it.control_names, it.times, vars.at(controls),
            lower.at(controls), upper.at(controls));
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
    timeValues(0) = vars.at(initial_time);
    timeValues(1) = vars.at(final_time);

    casadi::DM timeLower(2, 1);
    timeLower(0) = lower.at(initial_time);
    timeLower(1) = lower.at(final_time);

    casadi::DM timeUpper(2, 1);
    timeUpper(0) = upper.at(initial_time);
    timeUpper(1) = upper.at(final_time);

    printParameterBounds(
            "Time bounds", time_names, timeValues, timeLower, timeUpper);
    printParameterBounds("Parameter bounds", it.parameter_names,
            vars.at(parameters), lower.at(parameters), upper.at(parameters));

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
//    ss << "\nKinematic constraints:";
//    std::vector<std::string> kinconNames =
//            m_problem.createKinematicConstraintEquationNames();
//    if (kinconNames.empty()) {
//        ss << " none" << std::endl;
//    } else {
//        maxNameLength = 0;
//        updateMaxNameLength(kinconNames);
//        ss << "\n  L2 norm across mesh, max abs value (L1 norm), time of "
//                  "max "
//                  "abs"
//               << std::endl;
//        row.resize(1, m_numMeshPoints);
//        {
//            for (int ikc = 0; ikc < (int)constraints.kinematic.rows(); ++ikc) {
//                row = constraints.kinematic(ikc, Slice());
//                const double L2 = casadi::DM::norm_2(row).scalar();
//                int argmax;
//                double max = calcL1Norm(row, argmax);
//                const double L1 = max;
//                const double time_of_max = it.times(argmax).scalar();
//
//                std::string label = kinconNames.at(ikc);
//                ss << std::setfill('0') << std::setw(2) << ikc << ":"
//                       << std::setfill(' ') << std::setw(maxNameLength) << label
//                       << spacer << std::setprecision(2) << std::scientific
//                       << std::setw(9) << L2 << spacer << L1 << spacer
//                       << std::setprecision(6) << std::fixed << time_of_max
//                       << std::endl;
//            }
//        }
//        ss << "Kinematic constraint values at each mesh point:"
//               << std::endl;
//        ss << "      time  ";
//        for (int ipc = 0; ipc < (int)kinconNames.size(); ++ipc) {
//            ss << std::setw(9) << ipc << "  ";
//        }
//        ss << std::endl;
//        for (int imesh = 0; imesh < m_numMeshPoints; ++imesh) {
//            ss << std::setfill('0') << std::setw(3) << imesh << "  ";
//            ss.fill(' ');
//            ss << std::setw(9) << it.times(imesh).scalar() << "  ";
//            for (int ikc = 0; ikc < (int)kinconNames.size(); ++ikc) {
//                const auto& value = constraints.kinematic(ikc, imesh).scalar();
//                ss << std::setprecision(2) << std::scientific
//                       << std::setw(9) << value << "  ";
//            }
//            ss << std::endl;
//        }
//    }

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
    for (int i = 0; i < (int)inputs.size(); ++i) {
        if (inputs[i] == multibody_states) {
            const auto NQ = m_problem.getNumCoordinates();
            const auto NU = m_problem.getNumSpeeds();
            mxIn[i + 1] = 
                m_unscaledVars.at(states)(Slice(0, NQ + NU), timeIndices);
        } else if (inputs[i] == slacks) {
            mxIn[i + 1] = m_unscaledVars.at(inputs[i]);
        } else {
            mxIn[i + 1] = m_unscaledVars.at(inputs[i])(Slice(), timeIndices);
        }
    }
    if (&timeIndices == &m_gridIndices) {
        mxIn[mxIn.size() - 1] = m_paramsTrajGrid;
    } else if (&timeIndices == &m_meshIndices) {
        mxIn[mxIn.size() - 1] = m_paramsTrajMesh;
    } else if (&timeIndices == &m_meshInteriorIndices) {
        mxIn[mxIn.size() - 1] = m_paramsTrajMeshInterior;
    } else if (&timeIndices == &m_pathConstraintIndices) {
        mxIn[mxIn.size() - 1] = m_paramsTrajPathCon;
    } else if (&timeIndices == &m_projectionStateIndices) {
        mxIn[mxIn.size() - 1] = m_paramsTrajProjState;
    } else {
        OPENSIM_THROW(OpenSim::Exception, "Internal error.");
    }
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
