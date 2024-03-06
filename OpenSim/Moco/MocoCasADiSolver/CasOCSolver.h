#ifndef OPENSIM_CASOCSOLVER_H
#define OPENSIM_CASOCSOLVER_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoCasOCSolver.h                                                 *
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

#include "CasOCProblem.h"

namespace OpenSim {
class MocoCasADiSolver;
} // namespace OpenSim

namespace CasOC {

class Transcription;

/// Once you have built your CasOC::Problem, create a CasOC::Solver to configure
/// how you want to solve the problem, then invoke solve() to solve your
/// problem. This class assumes that the problem is solved using direct
/// collocation.
class Solver {
public:
    Solver(const Problem& problem) : m_problem(problem) {}
    void setNumMeshIntervals(int numMeshIntervals) {
        for (int i = 0; i < (numMeshIntervals + 1); ++i) {
            m_mesh.push_back(i / (double)(numMeshIntervals));
        }
    }
    void setMesh(std::vector<double> mesh) { m_mesh = std::move(mesh); }

    const std::vector<double>& getMesh() const { return m_mesh; }
    void setTranscriptionScheme(std::string scheme) {
        m_transcriptionScheme = std::move(scheme);
    }
    const std::string& getTranscriptionScheme() const {
        return m_transcriptionScheme;
    }
    std::string getDynamicsMode() const { return m_problem.getDynamicsMode(); }
    void setMinimizeLagrangeMultipliers(bool tf) {
        m_minimizeLagrangeMultipliers = tf;
    }
    void setScaleVariablesUsingBounds(bool value) {
        m_scaleVariablesUsingBounds = value;
    }
    bool getScaleVariablesUsingBounds() const {
        return m_scaleVariablesUsingBounds;
    }
    bool getMinimizeLagrangeMultipliers() const {
        return m_minimizeLagrangeMultipliers;
    }
    void setLagrangeMultiplierWeight(double weight) {
        m_lagrangeMultiplierWeight = weight;
    }
    double getLagrangeMultiplierWeight() const {
        return m_lagrangeMultiplierWeight;
    }
    void setImplicitMultibodyAccelerationBounds(Bounds bounds) {
        m_implicitMultibodyAccelerationBounds = bounds;
    }
    Bounds getImplicitMultibodyAccelerationBounds() const {
        return m_implicitMultibodyAccelerationBounds;
    }
    bool getMinimizeImplicitMultibodyAccelerations() const {
        return m_minimizeImplicitMultibodyAccelerations;
    }
    void setMinimizeImplicitMultibodyAccelerations(bool tf) {
        m_minimizeImplicitMultibodyAccelerations = tf;
    }
    double getImplicitMultibodyAccelerationsWeight() const {
        return m_implicitMultibodyAccelerationsWeight;
    }
    void setImplicitMultibodyAccelerationsWeight(double weight) {
        m_implicitMultibodyAccelerationsWeight = weight;
    }
    void setImplicitAuxiliaryDerivativeBounds(Bounds bounds) {
        m_implicitAuxiliaryDerivativeBounds = bounds;
    }
    Bounds getImplicitAuxiliaryDerivativeBounds() const {
        return m_implicitAuxiliaryDerivativeBounds;
    }
    bool getMinimizeImplicitAuxiliaryDerivatives() const {
        return m_minimizeImplicitAuxiliaryDerivatives;
    }
    void setMinimizeImplicitAuxiliaryDerivatives(bool tf) {
        m_minimizeImplicitAuxiliaryDerivatives = tf;
    }
    double getImplicitAuxiliaryDerivativesWeight() const {
        return m_implicitAuxiliaryDerivativesWeight;
    }
    void setImplicitAuxiliaryDerivativesWeight(double weight) {
        m_implicitAuxiliaryDerivativesWeight = weight;
    }

    /// Whether or not to constrain control values at mesh interval midpoints
    /// by linearly interpolating control values from mesh interval endpoints.
    /// @note Only applies to Hermite-Simpson collocation.
    void setInterpolateControlMidpoints(bool tf) {
        m_interpolateControlMidpoints = tf;
    }
    bool getInterpolateControlMidpoints() const {
        return m_interpolateControlMidpoints;
    }

    /// Whether or not to enforce path constraints at mesh interval midpoints.
    /// @note Only applies to Hermite-Simpson collocation.
    /// @note Does not apply to implicit dynamics residuals, as these are
    ///       always enforced at mesh interval midpoints.
    void setEnforcePathConstraintMidpoints(bool tf) {
        m_enforcePathConstraintMidpoints = tf;
    }
    bool getEnforcePathConstraintMidpoints() const {
        return m_enforcePathConstraintMidpoints;
    }

    void setOptimSolver(std::string optimSolver) {
        m_optimSolver = std::move(optimSolver);
    }
    const std::string getOptimSolver() const { return m_optimSolver; }

    /// The finite difference scheme to be set on all CasOC::Function objects.
    /// @note Default is 'central'.
    void setFiniteDifferenceScheme(const std::string& scheme) {
        m_finite_difference_scheme = scheme;
    }
    /// @copydoc setFiniteDifferenceScheme()
    std::string getFiniteDifferenceScheme() const {
        return m_finite_difference_scheme;
    }

    void setCallbackInterval(int callbackInterval) {
        m_callbackInterval = callbackInterval;
    }

    int getCallbackInterval() const { return m_callbackInterval; }
    /// "none" to use block sparsity (treat all CasOC::Function%s as dense;
    /// default), "initial-guess", or "random".
    void setSparsityDetection(const std::string& setting);
    /// If sparsity detection is "random", use this number of random iterates
    /// to determine sparsity.
    void setSparsityDetectionRandomCount(int count);

    /// If this is set to a non-empty string, the sparsity patterns of the
    /// optimization problem derivatives are written to files whose names use
    /// `setting` as a prefix.
    void setWriteSparsity(const std::string& setting) {
        m_write_sparsity = setting;
    }
    std::string getWriteSparsity() const { return m_write_sparsity; }

    /// Use this to tell CasADi to evaluate differential-algebraic equations,
    /// path constraints, integrands, etc. in parallel across grid points.
    /// "parallelism" is passed on directly to
    /// the "parallelism" argument of casadi::MX::map(). CasADi supports
    /// "serial", "openmp", "thread", and perhaps some other options.
    void setParallelism(std::string parallelism, int numThreads);
    std::pair<std::string, int> getParallelism() const {
        return std::make_pair(m_parallelism, m_numThreads);
    }

    void setPluginOptions(casadi::Dict opts) {
        m_pluginOptions = std::move(opts);
    }
    const casadi::Dict& getPluginOptions() const { return m_pluginOptions; }

    void setSolverOptions(casadi::Dict solverOptions) {
        m_solverOptions = std::move(solverOptions);
    }
    const casadi::Dict getSolverOptions() const { return m_solverOptions; }

    /// The contents of this iterate depends on the transcription scheme.
    Iterate createInitialGuessFromBounds() const;
    /// The contents of this iterate depends on the transcription scheme.
    Iterate createRandomIterateWithinBounds() const;

    Solution solve(const Iterate& guess) const;

private:
    std::unique_ptr<Transcription> createTranscription() const;

    const Problem& m_problem;
    std::vector<double> m_mesh;
    std::string m_transcriptionScheme = "hermite-simpson";
    bool m_scaleVariablesUsingBounds = false;
    bool m_minimizeLagrangeMultipliers = false;
    double m_lagrangeMultiplierWeight = 1.0;
    bool m_minimizeImplicitMultibodyAccelerations = false;
    double m_implicitMultibodyAccelerationsWeight = 1.0;
    bool m_minimizeImplicitAuxiliaryDerivatives = false;
    double m_implicitAuxiliaryDerivativesWeight = 1.0;
    bool m_interpolateControlMidpoints = true;
    bool m_enforcePathConstraintMidpoints = false;
    Bounds m_implicitMultibodyAccelerationBounds;
    Bounds m_implicitAuxiliaryDerivativeBounds;
    std::string m_finite_difference_scheme = "central";
    std::string m_sparsity_detection = "none";
    std::string m_write_sparsity;
    int m_callbackInterval = 0;
    int m_sparsity_detection_random_count = 3;
    std::string m_parallelism = "serial";
    int m_numThreads = 1;
    casadi::Dict m_pluginOptions;
    casadi::Dict m_solverOptions;
    std::string m_optimSolver;
};

} // namespace CasOC

#endif // OPENSIM_CASOCSOLVER_H
