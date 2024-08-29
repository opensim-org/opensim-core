#ifndef OPENSIM_CASOCTRANSCRIPTION_H
#define OPENSIM_CASOCTRANSCRIPTION_H
/* -------------------------------------------------------------------------- *
 * OpenSim: CasOCTranscription.h                                              *
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

#include "CasOCSolver.h"

namespace CasOC {

/// This is the base class for transcription schemes that convert a
/// CasOC::Problem into a general nonlinear programming problem. If you are
/// creating a new derived class, make sure to override all virtual functions
/// and obey the settings that the user specified in the CasOC::Solver.
class Transcription {
public:
    Transcription(const Solver& solver, const Problem& problem)
            : m_solver(solver), m_problem(problem) {}
    virtual ~Transcription() = default;
    Iterate createInitialGuessFromBounds() const;
    /// Use the provided random number generator to generate an iterate.
    /// Random::Uniform is used if a generator is not provided. The generator
    /// should produce numbers with [-1, 1].
    Iterate createRandomIterateWithinBounds(
            const SimTK::Random* = nullptr) const;
    template <typename T>
    T createTimes(const T& initial_time, const T& final_time) const {
        return (final_time(0) - initial_time(0)) * m_grid + initial_time(0);
    }
    casadi::DM createQuadratureCoefficients() const {
        return createQuadratureCoefficientsImpl();
    }
    casadi::DM createMeshIndices() const {
        casadi::DM meshIndices = createMeshIndicesImpl();
        const auto shape = meshIndices.size();
        OPENSIM_THROW_IF(shape.first != 1 || shape.second != m_numGridPoints,
                OpenSim::Exception,
                "createMeshIndicesImpl() must return a row vector of shape "
                "length [1, {}], but a matrix of shape [{}, {}] was returned.",
                m_numGridPoints, shape.first, shape.second);
        OPENSIM_THROW_IF(!SimTK::isNumericallyEqual(
                                 casadi::DM::sum2(meshIndices).scalar(),
                                 m_numMeshPoints),
                OpenSim::Exception,
                "Internal error: sum of mesh indices should be the number of "
                "mesh points.");

        return meshIndices;
    }
    casadi::DM createControlIndices() const {
        casadi::DM controlIndices = createControlIndicesImpl();
        const auto shape = controlIndices.size();
        OPENSIM_THROW_IF(shape.first != 1 || shape.second != m_numGridPoints,
                OpenSim::Exception,
                "createControlIndicesImpl() must return a row vector of shape "
                "length [1, {}], but a matrix of shape [{}, {}] was returned.",
                m_numGridPoints, shape.first, shape.second);

        return controlIndices;
    }

    Solution solve(const Iterate& guessOrig);

protected:
    /// This must be called in the constructor of derived classes so that
    /// overridden virtual methods are accessible to the base class. This
    /// implementation allows initialization to occur during construction,
    /// avoiding an extra call on the instantiated object.
    /// TODO control points
    void createVariablesAndSetBounds(const casadi::DM& grid,
            int numDefectsPerMeshInterval,
            int numPointsPerMeshInterval);

    /// We assume all functions depend on time and parameters.
    /// "inputs" is prepended by time and postpended (?) by parameters.
    casadi::MXVector evalOnTrajectory(const casadi::Function& pointFunction,
            const std::vector<Var>& inputs,
            const casadi::Matrix<casadi_int>& timeIndices) const;

    template <typename TRow, typename TColumn>
    void setVariableBounds(Var var, const TRow& rowIndices,
            const TColumn& columnIndices, const Bounds& bounds) {
        if (bounds.isSet()) {
            const auto& lower = bounds.lower;
            m_lowerBounds[var](rowIndices, columnIndices) = lower;
            const auto& upper = bounds.upper;
            m_upperBounds[var](rowIndices, columnIndices) = upper;
        } else {
            const auto inf = std::numeric_limits<double>::infinity();
            m_lowerBounds[var](rowIndices, columnIndices) = -inf;
            m_upperBounds[var](rowIndices, columnIndices) = inf;
        }
    }

    template <typename TRow, typename TColumn>
    void setVariableScaling(Var key, const TRow& rowIndices,
        const TColumn& columnIndices, const Bounds& bounds) {
        if (m_solver.getScaleVariablesUsingBounds()) {
            const auto& lower = bounds.lower;
            const auto& upper = bounds.upper;
            double dilate = upper - lower;
            double shift;
            if (std::isinf(dilate) || std::isnan(dilate)) {
                dilate = 1;
                shift = 0;
            } else if (dilate == 0) {
                dilate = 1;
                shift = upper;
            } else {
                shift = -0.5 * (upper + lower);
            }
            m_scale.at(key)(rowIndices, columnIndices) = dilate;
            m_shift.at(key)(rowIndices, columnIndices) = shift;
        } else {
            m_scale.at(key)(rowIndices, columnIndices) = 1;
            m_shift.at(key)(rowIndices, columnIndices) = 0;
        }
    }

    template <typename T>
    struct Constraints {
        T time;
        T parameters;
        T defects;
        T multibody_residuals;
        T auxiliary_residuals;
        T kinematic;
        T kinematic_udoterr;
        T integral;
        std::vector<T> endpoint;
        std::vector<T> path;
        T projection;
    };
    void printConstraintValues(const Iterate& it,
            const Constraints<casadi::DM>& constraints,
            std::ostream& stream = std::cout) const;
    void printObjectiveBreakdown(const Iterate& it,
            const casadi::DM& objectiveTerms,
            std::ostream& stream = std::cout) const;

    const Solver& m_solver;
    const Problem& m_problem;
    int m_numGridPoints = -1;
    int m_numMeshPoints = -1;
    int m_numMeshIntervals = -1;
    int m_numMeshInteriorPoints = -1;
    int m_numDefectsPerMeshInterval = -1;
    int m_numPointsPerMeshInterval = -1;
    int m_numUDotErrorPoints = -1;
    int m_numControlPoints = -1;
    int m_numMultibodyResiduals = -1;
    int m_numAuxiliaryResiduals = -1;
    int m_numIntegrals = -1;
    int m_numConstraints = -1;
    int m_numPathConstraintPoints = -1;
    int m_numProjectionStates = -1;
    casadi::DM m_grid;
    casadi::MX m_times;
    casadi::MX m_parameters;
    casadi::MX m_intervals;
    casadi::MX m_duration;

private:
    VectorVariablesMX m_scaledVectorVars;
    VariablesMX m_scaledVars;
    VariablesMX m_unscaledVars;
    VariablesDM m_lowerBounds;
    VariablesDM m_upperBounds;
    VariablesDM m_shift;
    VariablesDM m_scale;
    // These hold vectors of MX types, where each element of the vector
    // contains either the states or state derivatives needed to calculate the
    // defect constraints for a single mesh interval.
    casadi::MXVector m_statesByMeshInterval;
    casadi::MXVector m_stateDerivativesByMeshInterval;

    casadi::DM m_meshIndicesMap;
    casadi::Matrix<casadi_int> m_gridIndices;
    casadi::Matrix<casadi_int> m_meshIndices;
    casadi::Matrix<casadi_int> m_meshInteriorIndices;
    casadi::Matrix<casadi_int> m_pathConstraintIndices;
    casadi::Matrix<casadi_int> m_controlIndices;
    casadi::Matrix<casadi_int> m_projectionStateIndices;
    casadi::Matrix<casadi_int> m_notProjectionStateIndices;

    // State derivatives.
    casadi::MX m_xdot;
    // State derivatives reserved for the Bordalba et al. (2023) kinematic
    // constraint method based on coordinate projection.
    casadi::MX m_xdot_projection;
    // The differences between the true states and the projection states when
    // using the Bordalba et al. (2023) kinematic constraint method.
    casadi::MX m_projectionStateDistances;


    casadi::MX m_objectiveTerms;
    std::vector<std::string> m_objectiveTermNames;

    Constraints<casadi::MX> m_constraints;
    Constraints<casadi::DM> m_constraintsLowerBounds;
    Constraints<casadi::DM> m_constraintsUpperBounds;

private:
    /// Override this function in your derived class to compute a vector of
    /// quadrature coeffecients (of length m_numGridPoints) required to set the
    /// the integral cost within transcribe().
    virtual casadi::DM createQuadratureCoefficientsImpl() const = 0;
    /// Override this function to specify the indicies in the grid where the
    /// mesh (or "knot") points lie.
    /// @note The returned vector must be a row vector of length m_numGridPoints
    /// with nonzero values at the mesh indices.
    virtual casadi::DM createMeshIndicesImpl() const = 0;
    /// Override this function to specify the indicies in the grid where the
    /// control points lie.
    /// @note The returned vector must be a row vector of length m_numGridPoints
    /// with nonzero values at the control indices.
    virtual casadi::DM createControlIndicesImpl() const = 0;
    /// Override this function in your derived class set the defect, kinematic,
    /// and path constraint errors required for your transcription scheme.
    virtual void calcDefectsImpl(const casadi::MXVector& x, 
            const casadi::MXVector& xdot, casadi::MX& defects) const = 0;
    /// Override this function in your derived class to interpolate controls
    /// for time points where control variables are not defined.
    virtual void calcInterpolatingControlsImpl(casadi::MX& controls) const {
        OPENSIM_THROW_IF(casadi::DM::all(createControlIndices()).scalar() == 0,
                OpenSim::Exception,
                "Must provide scheme for interpolating controls.");
    }
    /// Override this function in your derived class to interpolate controls
    /// for time points where control variables are not defined.
    virtual void calcInterpolatingControlsImpl(casadi::DM& controls) const {
        OPENSIM_THROW_IF(casadi::DM::all(createControlIndices()).scalar() == 0,
                OpenSim::Exception,
                "Must provide scheme for interpolating controls.");
    }
    /// Override this function to define the order of variables in the flattened
    /// variable vector passed to nlpsol(). Returns a vector whose elements are
    /// pairs of variable keys and trajectory indexes.
    virtual std::vector<std::pair<Var, int>> getVariableOrder() const = 0;

    void transcribe();
    void setObjectiveAndEndpointConstraints();
    void calcDefects() {
        calcDefectsImpl(m_statesByMeshInterval, 
                m_stateDerivativesByMeshInterval, m_constraints.defects);
    }
    template <typename T>
    void calcInterpolatingControls(Variables<T>& vars) const {
        if (m_solver.getInterpolateControlMeshInteriorPoints()) {
            calcInterpolatingControlsImpl(vars.at(controls));
        }
    }

    /// Convert the map of variables into a column vector, for passing onto
    /// nlpsol(), etc.
    casadi::MX flattenVariables(const VectorVariablesMX& vars) const {
        std::vector<casadi::MX> stdvec;
        auto varOrder = getVariableOrder();
        for (const auto& kv : varOrder) {
            stdvec.push_back(vars.at(kv.first)[kv.second]);
        }

        return casadi::MX::vertcat(stdvec);
    }

    /// Convert the map of variables into a column vector, for passing onto
    /// nlpsol(), etc.
    casadi::DM flattenVariables(const VariablesDM& vars) const {
        std::vector<casadi::DM> stdvec;
        auto varOrder = getVariableOrder();
        for (const auto& kv : varOrder) {
            if (m_scaledVars.at(kv.first).rows()) {
                stdvec.push_back(vars.at(kv.first)(casadi::Slice(), kv.second));
            }
        }

        return casadi::DM::vertcat(stdvec);
    }

    /// Convert the 'x' column vector into separate variables.
    VariablesDM expandVariables(const casadi::DM& x) const {
        VariablesDM out;
        using casadi::Slice;
        casadi_int offset = 0;
        for (const auto& kv : m_scaledVars) {
            out[kv.first] = casadi::DM::zeros(kv.second.rows(),
                    kv.second.columns());
        }

        auto varOrder = getVariableOrder();
        for (const auto& kv : varOrder) {
            const auto& var = kv.first;
            const auto& index = kv.second;
            casadi_int size = m_scaledVars.at(var).rows();
            if (size) {
                out[var](Slice(), index) = casadi::DM::reshape(
                        x(Slice(offset, offset + size)), size, 1);
                offset += size;
            }
        }

        return out;
    }

    /// unscaled = (upper - lower) * scaled - 0.5 * (upper + lower);
    template <typename T>
    Variables<T> unscaleVariables(const Variables<T>& scaledVars) const {
        using casadi::DM;
        Variables<T> out;

        for (const auto& kv : scaledVars) {
            const auto& key = kv.first;
            const auto& scaled = scaledVars.at(key);
            const auto& numCols = scaled.columns();
            // The shift and scale are column vectors. For appropriate
            // elementwise math, we repeat the column to match the number of
            // columns for this key.
            const auto& shift = DM::repmat(m_shift.at(key), 1, numCols);
            const auto& scale = DM::repmat(m_scale.at(key), 1, numCols);
            out[key] = scaled * scale + shift;
        }
        return out;
    }

    /// scaled = [unscaled + 0.5 * (upper + lower)] / (upper - lower)
    template <typename T>
    Variables<T> scaleVariables(const Variables<T>& unscaledVars) {
        using casadi::DM;
        Variables<T> out;

        for (const auto& kv : unscaledVars) {
            const auto& key = kv.first;
            const auto& unscaled = unscaledVars.at(key);
            const auto& numCols = unscaled.columns();
            // The shift and scale are column vectors. For appropriate
            // elementwise math, we repeat the column to match the number of
            // columns for this key.
            const auto& shift = DM::repmat(m_shift.at(key), 1, numCols);
            const auto& scale = DM::repmat(m_scale.at(key), 1, numCols);
            out[key] = (unscaled - shift) / scale;
        }
        return out;
    }

    /// Flatten the constraints into a row vector, keeping constraints
    /// grouped together by time. Organizing the sparsity of the Jacobian
    /// this way might have benefits for sparse linear algebra.
    template <typename T>
    T flattenConstraints(const Constraints<T>& constraints) const {
        T flat = T(casadi::Sparsity::dense(m_numConstraints, 1));

        int iflat = 0;
        auto copyColumn = [&flat, &iflat](const T& matrix, int columnIndex) {
            using casadi::Slice;
            if (matrix.rows()) {
                flat(Slice(iflat, iflat + matrix.rows())) =
                        matrix(Slice(), columnIndex);
                iflat += matrix.rows();
            }
        };

        // Trapezoidal sparsity pattern (mapping between flattened and expanded
        // constraints) for mesh intervals 0, 1 and 2: Endpoint constraints
        // depend on all time points through their integral.
        //
        //                   0    1    2    3
        //    endpoint       x    x    x    x
        //    defect_0       x    x
        //    projection_1        x
        //    path_1              x
        //    kinematic_1         x
        //    residual_1          x
        //    defect_1            x    x
        //    projection_2             x
        //    path_2                   x
        //    kinematic_2              x
        //    residual_2               x
        //    defect_2                 x    x
        //    projection_3                  x
        //    path_3                        x
        //    kinematic_3                   x
        //    residual_3                    x

        // Hermite-Simpson sparsity pattern for mesh intervals 0, 1 and 2.
        // '*' indicates additional non-zero entry when path constraint
        // mesh interior points are enforced. Note that acceleration-level 
        // kinematic constraints, "kinematic_udoterr_0", are only enforced at 
        // mesh interior points (e.g., 0.5, 1.5, 2.5) when using the Bordalba
        // et al. (2023) kinematic constraint method. This sparsity pattern also 
        // applies to the Legendre-Gauss and Legendre-Gauss-Radau transcription 
        // with polynomial degree equal to 1.
        //
        //                         0    0.5    1    1.5    2    2.5    3
        //    endpoint             x     x     x     x     x     x     x
        //    path_0               x     *
        //    kinematic_perr_0     x     
        //    kinematic_uerr_0     x     
        //    kinematic_udoterr_0  x     x
        //    residual_0           x     x
        //    defect_0             x     x     x
        //    interp_con_0         x     x     x
        //    projection_1                     x
        //    path_1                           x     *
        //    kinematic_perr_1                 x     
        //    kinematic_uerr_1                 x     
        //    kinematic_udoterr_1              x     x
        //    residual_1                       x     x
        //    defect_1                         x     x     x
        //    interp_con_1                     x     x     x
        //    projection_2                                 x
        //    path_2                                       x     *
        //    kinematic_perr_2                             x     
        //    kinematic_uerr_2                             x     
        //    kinematic_udoterr_2                          x     x
        //    residual_2                                   x     x
        //    defect_2                                     x     x     x
        //    interp_con_2                                 x     x     x
        //    projection_3                                             x
        //    path_3                                                   x
        //    kinematic_perr_3                                         x     
        //    kinematic_uerr_3                                         x     
        //    kinematic_udoterr_3                                      x
        //    residual_3                                               x
        //                         0    0.5    1    1.5    2    2.5    3

        

        // Constraints for each mesh interval.
        int N = m_numPointsPerMeshInterval - 1;
        int icon = 0;
        for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
            int igrid = imesh * N;

            // Time constraints.
            copyColumn(constraints.time, imesh);

            // Parameter constraints.
            copyColumn(constraints.parameters, imesh);

            // Defect constraints.
            copyColumn(constraints.defects, imesh);

            // Multibody and auxiliary residuals.
            for (int i = 0; i < N; ++i) {
                copyColumn(constraints.multibody_residuals, igrid + i);
                copyColumn(constraints.auxiliary_residuals, igrid + i);
            }

            // Kinematic constraints.
            copyColumn(constraints.kinematic, imesh);
            if (m_problem.isKinematicConstraintMethodBordalba2023()) {
                for (int i = 0; i < N; ++i) {
                    copyColumn(constraints.kinematic_udoterr, igrid + i);
                }
            }

            // Path constraints.
            if (m_solver.getEnforcePathConstraintMeshInteriorPoints()) {
                for (int i = 0; i < N; ++i) {
                    for (const auto& path : constraints.path) {
                        copyColumn(path, igrid + i);
                    }
                }
            } else {
                for (const auto& path : constraints.path) {
                    copyColumn(path, imesh);
                }
            }

            // Projection constraints.
            copyColumn(constraints.projection, imesh);

            // Integral constraints.
            if (imesh < m_numMeshIntervals - 1) {
                copyColumn(constraints.integral, imesh);
            }
        }

        // Final grid point.
        copyColumn(constraints.multibody_residuals, m_numGridPoints - 1);
        copyColumn(constraints.auxiliary_residuals, m_numGridPoints - 1);
        copyColumn(constraints.kinematic, m_numMeshPoints - 1);
        if (m_problem.isKinematicConstraintMethodBordalba2023()) {
            copyColumn(constraints.kinematic_udoterr, m_numGridPoints - 1);
        }
        if (m_solver.getEnforcePathConstraintMeshInteriorPoints()) {
            for (const auto& path : constraints.path) {
                copyColumn(path, m_numGridPoints - 1);
            }
        } else {
            for (const auto& path : constraints.path) {
                copyColumn(path, m_numMeshPoints - 1);
            }
        }

        for (const auto& endpoint : constraints.endpoint) {
            copyColumn(endpoint, 0);
        }

        OPENSIM_THROW_IF(iflat != m_numConstraints, OpenSim::Exception,
                "Internal error: final value of the index into the flattened "
                "constraints should be equal to the number of constraints.");
        return flat;
    }

    // Expand constraints that have been flattened into a Constraints struct.
    template <typename T>
    Constraints<T> expandConstraints(const T& flat) const {
        using casadi::Sparsity;

        // Allocate memory.
        auto init = [](int numRows, int numColumns) {
            return T(casadi::Sparsity::dense(numRows, numColumns));
        };
        Constraints<T> out;
        out.time = init(2, m_numMeshIntervals);
        out.parameters = init(m_problem.getNumParameters(), m_numMeshIntervals);
        out.defects = init(m_numDefectsPerMeshInterval, m_numMeshIntervals);
        out.multibody_residuals = init(m_numMultibodyResiduals,
                m_numGridPoints);
        out.auxiliary_residuals = init(m_numAuxiliaryResiduals,
                m_numGridPoints);
        int numQErr = m_problem.getNumQErr();
        int numUErr = m_problem.getNumUErr();
        int numUDotErr = m_problem.getNumUDotErr();
        int numKC = m_problem.isKinematicConstraintMethodBordalba2023() ?
                numQErr + numUErr : numQErr + numUErr + numUDotErr;
        out.kinematic = init(numKC,m_numMeshPoints);
        if (m_problem.isKinematicConstraintMethodBordalba2023()) {
            out.kinematic_udoterr = init(numUDotErr, m_numUDotErrorPoints);
        }
        out.projection = init(m_problem.getNumProjectionConstraintEquations(),
                m_numMeshIntervals);
        out.integral = init(m_numIntegrals, m_numMeshIntervals-1);
        out.endpoint.resize(m_problem.getEndpointConstraintInfos().size());
        for (int iec = 0; iec < (int)m_constraints.endpoint.size(); ++iec) {
            const auto& info = m_problem.getEndpointConstraintInfos()[iec];
            out.endpoint[iec] = init(info.num_outputs, 1);
        }
        out.path.resize(m_problem.getPathConstraintInfos().size());
        for (int ipc = 0; ipc < (int)m_constraints.path.size(); ++ipc) {
            const auto& info = m_problem.getPathConstraintInfos()[ipc];
            out.path[ipc] = init(info.size(), m_numPathConstraintPoints);
        }

        int iflat = 0;
        auto copyColumn = [&flat, &iflat](T& matrix, int columnIndex) {
            using casadi::Slice;
            if (matrix.rows()) {
                matrix(Slice(), columnIndex) =
                        flat(Slice(iflat, iflat + matrix.rows()));
                iflat += matrix.rows();
            }
        };

        // Constraints for each mesh interval.
        int N = m_numPointsPerMeshInterval - 1;
        int icon = 0;
        for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
            int igrid = imesh * N;

            // Time constraints.
            copyColumn(out.time, imesh);

            // Parameter constraints.
            copyColumn(out.parameters, imesh);

            // Defect constraints.
            copyColumn(out.defects, imesh);

            // Multibody and auxiliary residuals.
            for (int i = 0; i < N; ++i) {
                copyColumn(out.multibody_residuals, igrid + i);
                copyColumn(out.auxiliary_residuals, igrid + i);
            }

            // Kinematic constraints.
            copyColumn(out.kinematic, imesh);
            if (m_problem.isKinematicConstraintMethodBordalba2023()) {
                for (int i = 0; i < N; ++i) {
                    copyColumn(out.kinematic_udoterr, igrid + i);
                }
            }

            // Path constraints.
            if (m_solver.getEnforcePathConstraintMeshInteriorPoints()) {
                for (int i = 0; i < N; ++i) {
                    for (auto& path : out.path) {
                        copyColumn(path, igrid + i);
                    }
                }
            } else {
                for (auto& path : out.path) {
                    copyColumn(path, imesh);
                }
            }

            // Projection constraints.
            copyColumn(out.projection, imesh);

            // Integral constraints.
            if (imesh < m_numMeshIntervals - 1) {
                copyColumn(out.integral, imesh);
            }
        }

        // Final grid point.
        copyColumn(out.multibody_residuals, m_numGridPoints - 1);
        copyColumn(out.auxiliary_residuals, m_numGridPoints - 1);
        copyColumn(out.kinematic, m_numMeshPoints - 1);
        if (m_problem.isKinematicConstraintMethodBordalba2023()) {
            copyColumn(out.kinematic_udoterr, m_numGridPoints - 1);
        }
        if (m_solver.getEnforcePathConstraintMeshInteriorPoints()) {
            for (auto& path : out.path) {
                copyColumn(path, m_numGridPoints - 1);
            }
        } else {
            for (auto& path : out.path) {
                copyColumn(path, m_numMeshPoints - 1);
            }
        }

        for (auto& endpoint : out.endpoint) {
            copyColumn(endpoint, 0);
        }

        OPENSIM_THROW_IF(iflat != m_numConstraints, OpenSim::Exception,
                "Internal error: final value of the index into the flattened "
                "constraints should be equal to the number of constraints.");
        return out;
    }

    ObjectiveBreakdown expandObjectiveTerms(const casadi::DM& terms) const {
        ObjectiveBreakdown out;
        for (int io = 0; io < (int)m_objectiveTermNames.size(); ++io) {
            out.push_back(std::make_pair(
                    m_objectiveTermNames[io], terms(io).scalar()));
        }
        return out;
    }


    friend class NlpsolCallback;
};

} // namespace CasOC

#endif // OPENSIM_CASOCTRANSCRIPTION_H
