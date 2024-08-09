/* -------------------------------------------------------------------------- *
* OpenSim: MocoParameterExpressionGoal.h                                     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Allison John                                                    *
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

#include "MocoParameterExpressionGoal.h"
#include <lepton/Parser.h>
#include <lepton/ParsedExpression.h>

using namespace OpenSim;

void MocoParameterExpressionGoal::constructProperties() {
    constructProperty_expression("");
    constructProperty_parameters();
    constructProperty_variable_names();
}

void MocoParameterExpressionGoal::initializeOnModelImpl(const Model&) const {
    m_parameterProg = Lepton::Parser::parse(get_expression()).optimize().createProgram();
    setRequirements(1, 1);
}

void MocoParameterExpressionGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {

    std::map<std::string, double> parameterVars;
    for (int i=0; i<getProperty_variable_names().size(); ++i) {
        std::string variableName = getProperty_variable_names()[i];
        MocoParameter parameter = get_parameters(i);
        double parameterValue = parameter.getPropertyValue();
        parameterVars[variableName] = parameterValue;
    }
    integrand = m_parameterProg.evaluate(parameterVars);
}

void MocoParameterExpressionGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& values) const {
    values[0] = input.integral;
}

