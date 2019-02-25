#ifndef MOCO_CASOCSOLVER_H
#define MOCO_CASOCSOLVER_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCasOCSolver.h                                            *
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
    void setNumMeshPoints(int numMeshPoints) {
        m_numMeshPoints = numMeshPoints;
    }
    int getNumMeshPoints() const { return m_numMeshPoints; }

    void setTranscriptionScheme(std::string scheme) {
        m_transcriptionScheme = std::move(scheme);
    }
    const std::string& getTranscriptionScheme() const {
        return m_transcriptionScheme;
    }
    void setDynamicsMode(std::string dynamicsMode) {
        OPENSIM_THROW_IF(
                dynamicsMode != "explicit" && dynamicsMode != "implicit",
                OpenSim::Exception, "Invalid dynamics mode.");
        m_dynamicsMode = std::move(dynamicsMode);
    }
    const std::string& getDynamicsMode() const { return m_dynamicsMode; }
    bool isDynamicsModeImplicit() const { return m_dynamicsMode == "implicit"; }
    void setMinimizeLagrangeMultipliers(bool tf) {
        m_minimizeLagrangeMultipliers = tf;
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

    void setOptimSolver(std::string optimSolver) {
        m_optimSolver = std::move(optimSolver);
    }
    const std::string getOptimSolver() const { return m_optimSolver; }

    /// The finite difference scheme to be set on all CasOC::Function objects.
    /// @note Default is 'central'.
    // TODO move to solver class.
    void setFiniteDifferenceScheme(const std::string& scheme) {
        m_finite_difference_scheme = scheme;
    }
    /// @copydoc setFiniteDifferenceScheme()
    std::string getFiniteDifferenceScheme() const {
        return m_finite_difference_scheme;
    }

    /// "none" to use block sparsity (treat all CasOC::Function%s as dense;
    /// default), "initial-guess", or "random".
    void setSparsityDetection(const std::string& setting);
    /// If sparsity detection is "random", use this number of random iterates
    /// to determine sparsity.
    void setSparsityDetectionRandomCount(int count);

    void setWriteSparsity(const std::string& setting) {
        m_write_sparsity = setting;
    }
    std::string getWriteSparsity() const {
        return m_write_sparsity;
    }


    /// Use this to tell CasADi to evaluate the differential-algebraic equations
    /// in parallel across grid points. "parallelism" is passed on directly to
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
    int m_numMeshPoints;
    std::string m_transcriptionScheme = "trapezoidal";
    std::string m_dynamicsMode = "explicit";
    bool m_minimizeLagrangeMultipliers = false;
    double m_lagrangeMultiplierWeight = 1.0;
    std::string m_finite_difference_scheme = "central";
    std::string m_sparsity_detection = "none";
    std::string m_write_sparsity;
    int m_sparsity_detection_random_count = 3;
    std::string m_parallelism = "serial";
    int m_numThreads = 1;
    casadi::Dict m_pluginOptions;
    casadi::Dict m_solverOptions;
    std::string m_optimSolver;
};

} // namespace CasOC

#endif // MOCO_CASOCSOLVER_H
