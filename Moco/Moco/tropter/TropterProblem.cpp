/* -------------------------------------------------------------------------- *
 * OpenSim Moco: TropterProblem.cpp                                           *
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
#include "TropterProblem.h"

using namespace OpenSim;

template <typename T>
template <typename MocoTrajectoryType, typename tropIterateType>
MocoTrajectoryType MocoTropterSolver::TropterProblemBase<T>::
convertIterateTropterToMoco(const tropIterateType& tropSol) const {
    const auto& tropTime = tropSol.time;
    SimTK::Vector time((int)tropTime.size(), tropTime.data());
    const auto& state_names = tropSol.state_names;
    const auto& control_names = tropSol.control_names;

    const int& numMultipliers =
            this->m_total_mp + this->m_total_mv + this->m_total_ma;
    std::vector<std::string> multiplier_names(numMultipliers);
    std::copy_n(tropSol.adjunct_names.begin(), numMultipliers,
            multiplier_names.begin());

    const int numDerivatives =
            (int)tropSol.adjunct_names.size() - numMultipliers;
    assert(numDerivatives >= 0);
    std::vector<std::string> derivative_names(numDerivatives);
    std::copy_n(tropSol.adjunct_names.begin() + numMultipliers, numDerivatives,
            derivative_names.begin());

    const auto& slack_names = tropSol.diffuse_names;
    const int numSlacks = (int)slack_names.size();
    const auto& parameter_names = tropSol.parameter_names;

    int numTimes = (int)time.size();
    int numStates = (int)state_names.size();
    // Create and populate states matrix.
    SimTK::Matrix states;
    if (numStates) {
        states.resize(numTimes, numStates);
        for (int itime = 0; itime < numTimes; ++itime) {
            for (int istate = 0; istate < numStates; ++istate) {
                states(itime, istate) = tropSol.states(istate, itime);
            }
        }
    }
    int numControls = (int)control_names.size();
    // Instantiating a SimTK::Matrix with a zero row or column does not create
    // an empty matrix. For example,
    //      SimTK::Matrix controls(5, 0);
    // will create a matrix with five empty rows. So, for variables other than
    // states, only allocate memory if necessary. Otherwise, return an empty
    // matrix. This will prevent weird comparison errors between two iterates
    // that should be equal but have slightly different "empty" values.
    SimTK::Matrix controls;
    if (numControls) {
        controls.resize(numTimes, numControls);
        for (int itime = 0; itime < numTimes; ++itime) {
            for (int icontrol = 0; icontrol < numControls; ++icontrol) {
                controls(itime, icontrol) = tropSol.controls(icontrol, itime);
            }
        }
    }
    SimTK::Matrix multipliers;
    if (numMultipliers) {
        multipliers.resize(numTimes, numMultipliers);
        for (int itime = 0; itime < numTimes; ++itime) {
            for (int imultiplier = 0; imultiplier < numMultipliers;
                    ++imultiplier) {
                multipliers(itime, imultiplier) =
                        tropSol.adjuncts(imultiplier, itime);
            }
        }
    }
    SimTK::Matrix derivatives;
    if (numDerivatives) {
        derivatives.resize(numTimes, numDerivatives);
        for (int itime = 0; itime < numTimes; ++itime) {
            for (int iderivative = 0; iderivative < numDerivatives;
                    ++iderivative) {
                derivatives(itime, iderivative) =
                        tropSol.adjuncts(iderivative + numMultipliers, itime);
            }
        }
    }
    SimTK::Matrix slacks;
    if (numSlacks) {
        slacks.resize(numTimes, numSlacks);
        for (int itime = 0; itime < numTimes; ++itime) {
            for (int islack = 0; islack < numSlacks; ++islack) {
                slacks(itime, islack) = tropSol.diffuses(islack,
                    itime);
            }
        }
    }
    int numParameters = (int)parameter_names.size();
    // This produces an empty RowVector if numParameters is zero.
    SimTK::RowVector parameters(numParameters, tropSol.parameters.data());

    // Create iterate.
    MocoTrajectoryType mocoIter(time, state_names, control_names, multiplier_names,
                        derivative_names, parameter_names, states, controls,
                        multipliers, derivatives, parameters);
    // Append slack variables.
    for (int i = 0; i < numSlacks; ++i) {
        mocoIter.appendSlack(slack_names[i], slacks.col(i));
    }
    return mocoIter;
}

template <typename T>
OpenSim::MocoTrajectory
MocoTropterSolver::TropterProblemBase<T>::convertToMocoTrajectory(
        const tropter::Iterate& tropIter) const {
    using OpenSim::MocoTrajectory;
    return convertIterateTropterToMoco<MocoTrajectory, tropter::Iterate>(tropIter);
}

template <typename T>
OpenSim::MocoSolution MocoTropterSolver::TropterProblemBase<T>::
convertToMocoSolution(const tropter::Solution& tropSol) const {
    // TODO enhance when solution contains more info than iterate.
    using OpenSim::MocoSolution;
    using tropter::Solution;
    return convertIterateTropterToMoco<MocoSolution, Solution>(tropSol);
}

template <typename T>
tropter::Iterate MocoTropterSolver::TropterProblemBase<T>::
convertToTropterIterate(const OpenSim::MocoTrajectory& mocoIter) const {
    tropter::Iterate tropIter;
    if (mocoIter.empty()) return tropIter;

    using Eigen::Map;
    using Eigen::RowVectorXd;
    using Eigen::MatrixXd;
    using Eigen::VectorXd;

    const auto& time = mocoIter.getTime();
    tropIter.time = Map<const RowVectorXd>(&time[0], time.size());

    tropIter.state_names = mocoIter.getStateNames();
    tropIter.control_names = mocoIter.getControlNames();
    tropIter.adjunct_names = mocoIter.getMultiplierNames();
    const auto& derivativeNames = mocoIter.getDerivativeNames();
    tropIter.adjunct_names.insert(tropIter.adjunct_names.end(),
            derivativeNames.begin(), derivativeNames.end());
    tropIter.diffuse_names = mocoIter.getSlackNames();
    tropIter.parameter_names = mocoIter.getParameterNames();

    int numTimes = (int)time.size();
    int numStates = (int)tropIter.state_names.size();
    int numControls = (int)tropIter.control_names.size();
    int numMultipliers = (int)mocoIter.getMultiplierNames().size();
    int numDerivatives = (int)derivativeNames.size();
    int numDiffuses = (int)tropIter.diffuse_names.size();
    int numParameters = (int)tropIter.parameter_names.size();
    const auto& states = mocoIter.getStatesTrajectory();
    const auto& controls = mocoIter.getControlsTrajectory();
    const auto& multipliers = mocoIter.getMultipliersTrajectory();
    const auto& derivatives = mocoIter.getDerivativesTrajectory();
    const auto& slacks = mocoIter.getSlacksTrajectory();
    const auto& parameters = mocoIter.getParameters();
    // Moco's matrix is numTimes x numStates;
    // tropter's is numStates x numTimes.
    if (numStates) {
        tropIter.states = Map<const MatrixXd>(
                &states(0, 0), numTimes, numStates).transpose();
    } else {
        tropIter.states.resize(numStates, numTimes);
    }
    if (numControls) {
        tropIter.controls = Map<const MatrixXd>(
                &controls(0, 0), numTimes, numControls).transpose();
    } else {
        tropIter.controls.resize(numControls, numTimes);
    }

    tropIter.adjuncts.resize(numMultipliers + numDerivatives, numTimes);
    if (numMultipliers) {
        tropIter.adjuncts.topRows(numMultipliers) = Map<const MatrixXd>(
                &multipliers(0, 0), numTimes, numMultipliers).transpose();
    }
    if (numDerivatives) {
        tropIter.adjuncts.bottomRows(numDerivatives) = Map<const MatrixXd>(
                &derivatives(0, 0), numTimes, numDerivatives).transpose();
    }
    if (numDiffuses) {
        tropIter.diffuses = Map<const MatrixXd>(
            &slacks(0, 0), numTimes, numDiffuses).transpose();
    } else {
        tropIter.diffuses.resize(numDiffuses, numTimes);
    }
    if (numParameters) {
        tropIter.parameters = Map<const VectorXd>(
                &parameters(0), numParameters);
    } else {
        tropIter.parameters.resize(numParameters);
    }
    return tropIter;
}

// Explicit template instantiations.
// ---------------------------------
template class MocoTropterSolver::TropterProblemBase<double>;
