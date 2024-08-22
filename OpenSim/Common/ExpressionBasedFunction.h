#ifndef OPENSIM_EXPRESSION_BASED_FUNCTION_H_
#define OPENSIM_EXPRESSION_BASED_FUNCTION_H_
/* -------------------------------------------------------------------------- *
 *                    OpenSim:  ExpressionBasedFunction.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "osimCommonDLL.h"
#include "Function.h"
namespace OpenSim {

class OSIMCOMMON_API ExpressionBasedFunction : public Function {
    OpenSim_DECLARE_CONCRETE_OBJECT(ExpressionBasedFunction, Function);

public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(expression, std::string, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY(variables, std::string, "TODO");

//==============================================================================
// METHODS
//==============================================================================

    /** Default constructor. */
    ExpressionBasedFunction() { constructProperties(); }

    /** Convenience constructor.
     *  
     * @param expression The expression that defines this Function.
     * @param variables The variables that the expression is a function of.
     */
    ExpressionBasedFunction(std::string expression, 
            const std::vector<std::string>& variables) {
        constructProperties();
        set_expression(std::move(expression));
        setVariables(variables);
    }

    /**
     * The expression that defines this Function. The expression should be a 
     * mathematical expression that is a function of the variables defined via
     * the 'variables' property.
     * 
     * @note The expression cannot contain any whitespace characters.
     */
    void setExpression(std::string expression) {
        set_expression(std::move(expression));
    }
    /// @copydoc setExpression()
    const std::string& getExpression() const {
        return get_expression();
    }

    /**
     * The variables that the expression is a function of. The variables should
     * be defined as a list of strings.
     */
    void setVariables(const std::vector<std::string>& variables) {
        for (const auto& var : variables) {
            append_variables(var);
        }
    }
    /// @copydoc setVariables()
    std::vector<std::string> getVariables() const {
        std::vector<std::string> variables;
        for (int i = 0; i < getProperty_variables().size(); ++i) {
            variables.push_back(get_variables(i));
        }
        return variables;
    }

    /**
     * Return a pointer to a SimTK::Function object that implements this
     * function.
     */
    SimTK::Function* createSimTKFunction() const override;

private:
    void constructProperties() {
        constructProperty_expression("");
        constructProperty_variables();
    }
};



} // namespace OpenSim

#endif  // OPENSIM_EXPRESSION_BASED_FUNCTION_H_