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

/** 
 * A function based on a user-defined mathematical expression.
 * 
 * This class allows users to define a function based on a mathematical
 * expression (e.g., "x*sqrt(y-8)"). The expression can be a function of any 
 * number of independent variables. The expression is parsed and evaluated using 
 * the Lepton library.
 * 
 * Set the expression using setExpression(). Any variables used in the 
 * expression must be explicitly defined using setVariables(). This 
 * implementation allows computation of first-order derivatives only.
 * 
 * # Creating Expressions
 * 
 * Expressions can contain variables, constants, operations, parentheses, commas, 
 * spaces, and scientific "e" notation. The full list of supported operations is: 
 * sqrt, exp, log, sin, cos, sec, csc, tan, cot, asin, acos, atan, sinh, cosh, 
 * tanh, erf, erfc, step, delta, square, cube, recip, min, max, abs, +, -, *, /, 
 * and ^. 
 */
class OSIMCOMMON_API ExpressionBasedFunction : public Function {
    OpenSim_DECLARE_CONCRETE_OBJECT(ExpressionBasedFunction, Function);

public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(expression, std::string, 
            "The mathematical expression defining this Function.");
    OpenSim_DECLARE_LIST_PROPERTY(variables, std::string, 
            "The independent variables used by this Function's expression.");

//==============================================================================
// METHODS
//==============================================================================

    /** Default constructor. */
    ExpressionBasedFunction() { constructProperties(); }

    /** Convenience constructor.
     *  
     * @param expression The expression that defines this Function.
     * @param variables The independent variable names of this expression.
     */
    ExpressionBasedFunction(std::string expression, 
            const std::vector<std::string>& variables) {
        constructProperties();
        set_expression(std::move(expression));
        setVariables(variables);
    }

    /**
     * The mathematical expression that defines this Function. The expression 
     * should be a function of the variables defined via setVariables().
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
     * The independent variable names of this expression. The variables names 
     * should be unique. The input vector passed to calcValue() and 
     * calcDerivative() should be in the same order as the variables defined
     * here.
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