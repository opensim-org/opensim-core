/* -------------------------------------------------------------------------- *
 * OpenSim: MocoOutputGoal.cpp                                                *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

#include "MocoOutputGoal.h"

using namespace OpenSim;

void MocoOutputGoal::constructProperties() {
    constructProperty_output_path("");
    constructProperty_divide_by_displacement(false);
    constructProperty_divide_by_mass(false);
    constructProperty_exponent(1);
}

void MocoOutputGoal::initializeOnModelImpl(const Model& output) const {
    OPENSIM_THROW_IF_FRMOBJ(get_output_path().empty(), Exception,
            "No output_path provided.");
    std::string componentPath;
    std::string outputName;
    std::string channelName;
    std::string alias;
    AbstractInput::parseConnecteePath(
            get_output_path(), componentPath, outputName, channelName, alias);
    const auto& component = getModel().getComponent(componentPath);
    const auto* abstractOutput = &component.getOutput(outputName);

    if (dynamic_cast<const Output<double>*>(abstractOutput)) {
        m_data_type = Type_double;
    } else if (dynamic_cast<const Output<SimTK::Vec3>*>(abstractOutput)) {
        m_data_type = Type_Vec3;
    } else {
        OPENSIM_THROW_FRMOBJ(Exception,
                "Data type of specified model output not supported.");
    }

    m_output.reset(abstractOutput);

    OPENSIM_THROW_IF_FRMOBJ(
            get_exponent() < 1, Exception, "Exponent must be 1 or greater.");
    int exponent = get_exponent();

    // The pow() function gives slightly different results than x * x. On Mac,
    // using x * x requires fewer solver iterations.
    if (exponent == 1) {
        m_power_function = [](const double& x) { return x; };
    } else if (exponent == 2) {
        m_power_function = [](const double& x) { return x * x; };
    } else {
        m_power_function = [exponent](const double& x) {
            return pow(std::abs(x), exponent);
        };
    }

    setRequirements(1, 1, m_output->getDependsOnStage());
}

void MocoOutputGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    getModel().getSystem().realize(input.state, m_output->getDependsOnStage());
    double value = 0;
    if (m_data_type == Type_double) {
        value = static_cast<const Output<double>*>(m_output.get())
                        ->getValue(input.state);
    } else if (m_data_type == Type_Vec3) {
        value = static_cast<const Output<SimTK::Vec3>*>(m_output.get())
                        ->getValue(input.state).norm();
    }

    integrand = m_power_function(value);
}

void MocoOutputGoal::calcGoalImpl(
        const MocoGoal::GoalInput& input, SimTK::Vector& cost) const {
    cost[0] = input.integral;
    if (get_divide_by_displacement()) {
        cost[0] /=
                calcSystemDisplacement(input.initial_state, input.final_state);
    }
    if (get_divide_by_mass()) {
        cost[0] /= getModel().getTotalMass(input.initial_state);
    }
}
