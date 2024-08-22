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
#include "FunctionAdapter.h"

#include <lepton/ExpressionProgram.h>
#include <lepton/ParsedExpression.h>
#include <lepton/Parser.h>

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

    // CONSTRUCTION 
    ExpressionBasedFunction(const std::string& expression, 
            const std::vector<std::string>& variables) {
        constructProperties();
        set_expression(expression);
        for (const auto& var : variables) {
            append_variables(var);
        }

        Lepton::ParsedExpression parsedExpression = 
                Lepton::Parser::parse(expression).optimize();

        for (int i = 0; i < variables.size(); ++i) {
            Lepton::ParsedExpression diffExpression = 
                    parsedExpression.differentiate(variables[i]).optimize();
            m_derivativePrograms.push_back(diffExpression.createProgram());
        }

        m_valueProgram = parsedExpression.createProgram();
    }

    double calcValue(const SimTK::Vector& x) const override {
        std::map<std::string, double> vars;
        for (int i = 0; i < getProperty_variables().size(); ++i) {
            vars[get_variables(i)] = x[i];
        }
        return m_valueProgram.evaluate(vars);
    }

    double calcDerivative(const std::vector<int>& derivComponents,
            const SimTK::Vector& x) const override {
        std::map<std::string, double> vars;
        for (int i = 0; i < getProperty_variables().size(); ++i) {
            vars[get_variables(i)] = x[i];
        }

        return m_derivativePrograms[derivComponents[0]].evaluate(vars);
    }


    SimTK::Function* createSimTKFunction() const override {
        return new FunctionAdapter(*this);
    }

private:
    void constructProperties() {
        constructProperty_expression("");
        constructProperty_variables();
    }

    Lepton::ExpressionProgram m_valueProgram;
    std::vector<Lepton::ExpressionProgram> m_derivativePrograms;
};



} // namespace OpenSim

#endif  // OPENSIM_EXPRESSION_BASED_FUNCTION_H_