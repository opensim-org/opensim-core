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

#include <OpenSim/Common/Assertion.h>

using namespace OpenSim;

template <typename T>
template <typename MocoTrajectoryType, typename tropIterateType>
MocoTrajectoryType MocoTropterSolver::TropterProblemBase<T>::
convertIterateTropterToMoco(const tropIterateType& tropSol,
        std::vector<int> inputControlIndexes) const {
    const auto& tropTime = tropSol.time;
    SimTK::Vector time((int)tropTime.size(), tropTime.data());
    const auto& state_names = tropSol.state_names;
    const auto& all_control_names = tropSol.control_names;

    const int& numMultipliers =
            this->m_total_mp + this->m_total_mv + this->m_total_ma;
    std::vector<std::string> multiplier_names(numMultipliers);
    std::copy_n(tropSol.adjunct_names.begin(), numMultipliers,
            multiplier_names.begin());

    const int numDerivatives =
            (int)tropSol.adjunct_names.size() - numMultipliers;
    OPENSIM_ASSERT(numDerivatives >= 0);
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
    std::sort(inputControlIndexes.begin(), inputControlIndexes.end());
    int numTotalControls = (int)all_control_names.size();
    int numInputControls = (int)inputControlIndexes.size();
    int numControls = numTotalControls - numInputControls;
    OPENSIM_ASSERT(numControls >= 0);
    // Instantiating a SimTK::Matrix with a zero row or column does not create
    // an empty matrix. For example,
    //      SimTK::Matrix controls(5, 0);
    // will create a matrix with five empty rows. So, for variables other than
    // states, only allocate memory if necessary. Otherwise, return an empty
    // matrix. This will prevent weird comparison errors between two iterates
    // that should be equal but have slightly different "empty" values.
    SimTK::Matrix all_controls;
    SimTK::Matrix controls;
    SimTK::Matrix input_controls;
    std::vector<std::string> control_names;
    std::vector<std::string> input_control_names;
    if (numTotalControls) {
        all_controls.resize(numTimes, numTotalControls);
        for (int itime = 0; itime < numTimes; ++itime) {
            for (int i = 0; i < numTotalControls; ++i) {
                all_controls(itime, i) = tropSol.controls(i, itime);
            }
        }
        controls.resize(numTimes, numControls);
        input_controls.resize(numTimes, numInputControls);
        int ic = 0;
        int iic = 0;
        std::sort(inputControlIndexes.begin(), inputControlIndexes.end());
        for (int icontrol = 0; icontrol < numTotalControls; ++icontrol) {
            if (iic < numInputControls && inputControlIndexes[iic] == icontrol) 
            {
                input_controls.updCol(iic) = all_controls.col(icontrol);
                input_control_names.push_back(all_control_names[icontrol]);
                iic++;
            } else {
                controls.updCol(ic) = all_controls.col(icontrol);
                control_names.push_back(all_control_names[icontrol]);
                ic++;
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
    MocoTrajectoryType mocoIter(time, state_names, control_names, 
                        input_control_names, multiplier_names,
                        derivative_names, parameter_names, states, controls,
                        input_controls, multipliers, derivatives, parameters);

    // Append slack variables. Interpolate slack variables to remove NaNs.
    for (int i = 0; i < numSlacks; ++i) {
        mocoIter.appendSlack(slack_names[i], 
                interpolate(time, slacks.col(i), time, true, true));
    }
    return mocoIter;
}

template <typename T>
OpenSim::MocoTrajectory
MocoTropterSolver::TropterProblemBase<T>::convertToMocoTrajectory(
        const tropter::Iterate& tropIter, 
        std::vector<int> inputControlIndexes) const {
    using OpenSim::MocoTrajectory;
    return convertIterateTropterToMoco<MocoTrajectory, tropter::Iterate>(
            tropIter, inputControlIndexes);
}

template <typename T>
OpenSim::MocoSolution MocoTropterSolver::TropterProblemBase<T>::
convertToMocoSolution(const tropter::Solution& tropSol, 
        std::vector<int> inputControlIndexes) const {
    // TODO enhance when solution contains more info than iterate.
    using OpenSim::MocoSolution;
    using tropter::Solution;
    return convertIterateTropterToMoco<MocoSolution, Solution>(
            tropSol, inputControlIndexes);
}

template <typename T>
tropter::Iterate MocoTropterSolver::TropterProblemBase<T>::
convertToTropterIterate(const OpenSim::MocoTrajectory& mocoIter, 
        std::vector<int> inputControlIndexes) const {
    tropter::Iterate tropIter;
    if (mocoIter.empty()) return tropIter;

    using Eigen::Map;
    using Eigen::RowVectorXd;
    using Eigen::MatrixXd;
    using Eigen::VectorXd;

    const auto& time = mocoIter.getTime();
    tropIter.time = Map<const RowVectorXd>(&time[0], time.size());

    tropIter.state_names = mocoIter.getStateNames();
    const auto& controlNames = mocoIter.getControlNames();
    const auto& inputControlNames = mocoIter.getInputControlNames();
    tropIter.adjunct_names = mocoIter.getMultiplierNames();
    const auto& derivativeNames = mocoIter.getDerivativeNames();
    tropIter.adjunct_names.insert(tropIter.adjunct_names.end(),
            derivativeNames.begin(), derivativeNames.end());
    tropIter.diffuse_names = mocoIter.getSlackNames();
    tropIter.parameter_names = mocoIter.getParameterNames();

    int numTimes = (int)time.size();
    int numStates = (int)tropIter.state_names.size();
    int numControls = mocoIter.getNumControls();
    int numInputControls = mocoIter.getNumInputControls();
    int numTotalControls = numControls + numInputControls;
    int numMultipliers = (int)mocoIter.getMultiplierNames().size();
    int numDerivatives = (int)derivativeNames.size();
    int numDiffuses = (int)tropIter.diffuse_names.size();
    int numParameters = (int)tropIter.parameter_names.size();
    const auto& states = mocoIter.getStatesTrajectory();
    const auto& controls = mocoIter.getControlsTrajectory();
    const auto& inputControls = mocoIter.getInputControlsTrajectory();
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
    if (numTotalControls) {
        tropIter.controls.resize(numTotalControls, numTimes);
        int ic = 0;
        int iic = 0;
        std::sort(inputControlIndexes.begin(), inputControlIndexes.end());
        for (int i = 0; i < numTotalControls; ++i) {
            if (iic < numInputControls && inputControlIndexes[iic] == i) {
                tropIter.controls.row(i) = Map<const VectorXd>(
                        &inputControls(0, iic), numTimes, 1).transpose();
                tropIter.control_names.push_back(inputControlNames[iic]);
                iic++;
            } else {
                tropIter.controls.row(i) = Map<const VectorXd>(
                        &controls(0, ic), numTimes, 1).transpose();
                tropIter.control_names.push_back(controlNames[ic]);
                ic++;   
            }
        }
    } else {
        tropIter.controls.resize(numTotalControls, numTimes);
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
