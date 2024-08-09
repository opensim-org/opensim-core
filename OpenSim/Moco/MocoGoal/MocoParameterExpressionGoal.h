#ifndef OPENSIM_MOCOPARAMETEREXPRESSIONGOAL_H
#define OPENSIM_MOCOPARAMETEREXPRESSIONGOAL_H
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

#include "MocoGoal.h"
#include "OpenSim/Moco/MocoParameter.h"
#include <lepton/ExpressionProgram.h>

namespace OpenSim {

class OSIMMOCO_API MocoParameterExpressionGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoParameterExpressionGoal, MocoGoal);

public:
    MocoParameterExpressionGoal() { constructProperties(); }
    MocoParameterExpressionGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoParameterExpressionGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    void setExpression(std::string expression) {
        set_expression(expression);
    }

    void addParameter(MocoParameter& parameter, std::string variableName) {
        updProperty_parameters().appendValue(parameter);
        updProperty_variable_names().appendValue(variableName);
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, SimTK::Real& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;
    //void printDescriptionImpl() const override;

private:
    void constructProperties();
    OpenSim_DECLARE_PROPERTY(expression, std::string,
            "The expression string with variables q0-q9.");
    // consider a mapping from variable names to parameters instead
    OpenSim_DECLARE_LIST_PROPERTY(parameters, MocoParameter,
            "MocoParameters to use in the expression.");
    OpenSim_DECLARE_LIST_PROPERTY(variable_names, std::string,
            "Variable names of the MocoParameters to use in the expression.");

    mutable Lepton::ExpressionProgram m_parameterProg;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOPARAMETEREXPRESSIONGOAL_H
