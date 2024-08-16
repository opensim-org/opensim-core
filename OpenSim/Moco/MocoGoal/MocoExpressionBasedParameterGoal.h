#ifndef OPENSIM_MOCOEXPRESSIONBASEDPARAMETERGOAL_H
#define OPENSIM_MOCOEXPRESSIONBASEDPARAMETERGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoExpressionBasedParameterGoal.h                                *
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
#include <SimTKcommon/internal/ReferencePtr.h>

namespace OpenSim {
class Model;

/** Minimize an arithmetic expression of parameters. This goal supports any
number of MocoParameters that are combined into a single goal. The expression
string should match the Lepton (lightweight expression parser) format.

# Creating Expressions

Expressions can be any string that represents a mathematical expression, e.g.,
"x*sqrt(y-8)". Expressions can contain variables, constants, operations,
parentheses, commas, spaces, and scientific e notation. The full list of
operations (also in Lepton::Operation) is:
sqrt, exp, log, sin, cos, sec, csc, tan, cot, asin, acos, atan, sinh, cosh,
tanh, erf, erfc, step, delta, square, cube, recip, min, max, abs, as well as
+, -, *, /, and ^.

# Examples

@code
// create two parameters to include in the goal
MocoStudy study;
MocoProblem& mp = study.updProblem();
mp.setModel(createOscillatorTwoSpringsModel());
mp.setTimeBounds(0, FINAL_TIME);
auto* parameter = mp.addParameter("spring_stiffness", "spring1",
                                          "stiffness", MocoBounds(0, 100));
auto* parameter2 = mp.addParameter("spring2_stiffness", "spring2",
                                          "stiffness", MocoBounds(0, 100));
auto* spring_goal = mp.addGoal<MocoExpressionBasedParameterGoal>();
// minimum is when p + q = STIFFNESS
spring_goal->setExpression(fmt::format("square( p+q-{} )", STIFFNESS));
spring_goal->addParameter(*parameter, "p");
spring_goal->addParameter(*parameter2, "q");
@endcode

@ingroup mocogoal */
class OSIMMOCO_API MocoExpressionBasedParameterGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoExpressionBasedParameterGoal, MocoGoal);

public:
    MocoExpressionBasedParameterGoal() { constructProperties(); }
    MocoExpressionBasedParameterGoal(std::string name)
            : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoExpressionBasedParameterGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }
    MocoExpressionBasedParameterGoal(std::string name, double weight,
            std::string expression) : MocoGoal(std::move(name), weight) {
        constructProperties();
        set_expression(std::move(expression));
    }

    /** Set the mathematical expression to minimize. Variable names should match
    the names set with addParameter(). See Creating Expressions above for an
    explanation of Expressions. */
    void setExpression(std::string expression) {
        set_expression(std::move(expression));
    }

    /** Add parameters with variable names that match the variables in the
    expression string. All variables in the expression must have a corresponding
    parameter, but parameters with variables that are not in the expression are
    ignored. */
    void addParameter(const MocoParameter& parameter, std::string variableName) {
        append_parameters(parameter);
        append_variable_names(std::move(variableName));
    }

protected:
    void initializeOnModelImpl(const Model& model) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    void printDescriptionImpl() const override;

private:
    void constructProperties();

    /** Get the value of the property from its index in the property_refs vector.
    This will use m_data_types to get the type, and if it is a Vec type, it uses
    m_indices to get the element to return, both at the same index i.*/
    double getPropertyValue(int i) const;

    OpenSim_DECLARE_PROPERTY(expression, std::string,
            "The expression string with variables q0-q9.");
    OpenSim_DECLARE_LIST_PROPERTY(parameters, MocoParameter,
            "MocoParameters to use in the expression.");
    OpenSim_DECLARE_LIST_PROPERTY(variable_names, std::string,
            "Variable names of the MocoParameters to use in the expression.");

    mutable Lepton::ExpressionProgram m_program;
    // stores references to one property per parameter
    mutable std::vector<SimTK::ReferencePtr<const AbstractProperty>> m_property_refs;
    enum DataType {
      Type_double,
      Type_Vec3,
      Type_Vec6
    };
    mutable std::vector<DataType> m_data_types;
    mutable std::vector<int> m_indices;

};

} // namespace OpenSim

#endif // OPENSIM_MOCOPARAMETEREXPRESSIONGOAL_H
