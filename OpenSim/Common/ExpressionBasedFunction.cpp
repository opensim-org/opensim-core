/* -------------------------------------------------------------------------- *
 *                   OpenSim:  ExpressionBasedFunction.cpp                    *
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

#include "ExpressionBasedFunction.h"

#include <lepton/ExpressionProgram.h>
#include <lepton/ParsedExpression.h>
#include <lepton/Parser.h>
#include <lepton/Exception.h>

using namespace OpenSim;

class SimTKExpressionBasedFunction : public SimTK::Function {
public:
    SimTKExpressionBasedFunction(const std::string& expression, 
            const std::vector<std::string>& variables) :
        m_expression(expression), m_variables(variables) {

        // Check that the variable names are unique.
        std::set<std::string> uniqueVariables;
        for (const auto& variable : m_variables) {
            if (!uniqueVariables.insert(variable).second) {
                OPENSIM_THROW(Exception, 
                        fmt::format("Variable '{}' is defined more than once.", 
                        variable));
            }
        }

        // Create the expression programs for the value and its derivatives.
        Lepton::ParsedExpression parsedExpression = 
                Lepton::Parser::parse(m_expression).optimize();
        m_valueProgram = parsedExpression.createProgram();

        for (int i = 0; i < static_cast<int>(m_variables.size()); ++i) {
            Lepton::ParsedExpression diffExpression = 
                    parsedExpression.differentiate(m_variables[i]).optimize();
            m_derivativePrograms.push_back(diffExpression.createProgram());
        }

        try {
            std::map<std::string, double> vars;
            for (int i = 0; i < static_cast<int>(m_variables.size()); ++i) {
                vars[m_variables[i]] = 0;
            }
            m_valueProgram.evaluate(vars);

            for (int i = 0; i < static_cast<int>(m_variables.size()); ++i) {
                m_derivativePrograms[i].evaluate(vars);
            }
        } catch (Lepton::Exception& ex) {
            std::string msg = ex.what();
            std::string undefinedVar = msg.substr(32, msg.size() - 32);
            OPENSIM_THROW(Exception, 
                    fmt::format("Variable '{}' is not defined. Use "
                    "setVariables() to explicitly define this variable. Or, "
                    "remove it from the expression.", undefinedVar));
        }
    }

    SimTK::Real calcValue(const SimTK::Vector& x) const override {
        OPENSIM_ASSERT(x.size() == static_cast<int>(m_variables.size()));
        std::map<std::string, double> vars;
        for (int i = 0; i < static_cast<int>(m_variables.size()); ++i) {
            vars[m_variables[i]] = x[i];
        }
        return m_valueProgram.evaluate(vars);
    }

    SimTK::Real calcDerivative(const SimTK::Array_<int>& derivComponents, 
            const SimTK::Vector& x) const override {
        OPENSIM_ASSERT(x.size() == static_cast<int>(m_variables.size()));
        OPENSIM_ASSERT(derivComponents.size() == 1);
        if (derivComponents[0] < static_cast<int>(m_variables.size())) {
            std::map<std::string, double> vars;
            for (int i = 0; i < static_cast<int>(m_variables.size()); ++i) {
                vars[m_variables[i]] = x[i];
            }
            return m_derivativePrograms[derivComponents[0]].evaluate(vars);
        }
        return 0.0;
    }

    int getArgumentSize() const override { 
        return static_cast<int>(m_variables.size()); 
    }
    int getMaxDerivativeOrder() const override { return 1; }
    SimTKExpressionBasedFunction* clone() const override {
        return new SimTKExpressionBasedFunction(*this);
    }

private:
    std::string m_expression;
    std::vector<std::string> m_variables;
    Lepton::ExpressionProgram m_valueProgram;
    std::vector<Lepton::ExpressionProgram> m_derivativePrograms;
};

SimTK::Function* ExpressionBasedFunction::createSimTKFunction() const {
    std::vector<std::string> variables;
    for (int i = 0; i < getProperty_variables().size(); ++i) {
        variables.push_back(get_variables(i));
    }
    return new SimTKExpressionBasedFunction(get_expression(), variables);
}