/* -------------------------------------------------------------------------- *
 * OpenSim: MocoExpressionBasedParameterGoal.cpp                              *
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

#include "MocoExpressionBasedParameterGoal.h"

#include <lepton/Exception.h>
#include <lepton/ParsedExpression.h>
#include <lepton/Parser.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

void MocoExpressionBasedParameterGoal::constructProperties() {
    constructProperty_expression("");
    constructProperty_parameters();
    constructProperty_variables();
}

void MocoExpressionBasedParameterGoal::initializeOnModelImpl(const Model& model)
        const {
    OPENSIM_THROW_IF_FRMOBJ(get_expression().empty(), Exception,
            "The expression has not been set. Use setExpression().")
    m_program = Lepton::Parser::parse(get_expression()).optimize()
                .createProgram();
    setRequirements(0, 1, SimTK::Stage::Instance);

    for (int i = 0; i < getProperty_parameters().size(); i++) {
        // only taking the first one since they should all be the same value
        std::string componentPath = get_parameters(i).getComponentPaths()[0];
        const auto& component = model.getComponent(componentPath);
        const auto* ap = &component.getPropertyByName(
                                get_parameters(i).getPropertyName());
        m_property_refs.emplace_back(ap);

        // get the type and element of the property
        if (dynamic_cast<const Property<double>*>(ap)) {
            m_data_types.emplace_back(Type_double);
        } else {
            if (dynamic_cast<const Property<SimTK::Vec3>*>(ap)) {
                m_data_types.emplace_back(Type_Vec3);
                m_indices.emplace_back(get_parameters(i).getPropertyElement());
            }
            else if (dynamic_cast<const Property<SimTK::Vec6>*>(ap)) {
                m_data_types.emplace_back(Type_Vec6);
                m_indices.emplace_back(get_parameters(i).getPropertyElement());
            }
            else {
                OPENSIM_THROW_FRMOBJ(Exception,
                    "Data type of specified model property not supported.");
            }
        }
    }

    // test to make sure all variables are there
    try {
        std::map<std::string, double> parameterVars;
        for (int i = 0; i < getProperty_variables().size(); ++i) {
            parameterVars[get_variables(i)] = getPropertyValue(i);
        }
        m_program.evaluate(parameterVars);
    } catch (Lepton::Exception& ex) {
        const std::string msg = ex.what();
        if (msg.compare(0, 30, "No value specified for variable")) {
            std::string undefinedVar = msg.substr(32, msg.size() - 32);
            OPENSIM_THROW_FRMOBJ(Exception, 
                    fmt::format("Parameter variable '{}' is not defined. Use "
                    "addParameter() to explicitly define this variable. Or, "
                    "remove it from the expression.", undefinedVar));
        } else {
            OPENSIM_THROW_FRMOBJ(Exception, "Lepton parsing error: {}", msg);
        }
    }
}

double MocoExpressionBasedParameterGoal::getPropertyValue(int i) const {
    OPENSIM_ASSERT_FRMOBJ(m_property_refs.size() > i);
    const auto& propRef = m_property_refs[i];
    if (m_data_types[i] == Type_double) {
        return static_cast<const Property<double>*>(propRef.get())->getValue();
    }
    int elt = m_indices[i];
    if (m_data_types[i] == Type_Vec3) {
        return static_cast<const Property<SimTK::Vec3>*>(propRef.get())
                                                     ->getValue()[elt];
    }
    if (m_data_types[i] == Type_Vec6) {
        return static_cast<const Property<SimTK::Vec6>*>(propRef.get())
                                                     ->getValue()[elt];
    }
    OPENSIM_THROW_FRMOBJ(Exception, fmt::format(fmt::runtime(
            "Property at index {} is not of a recognized type."), i));
}

void MocoExpressionBasedParameterGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& values) const {
    std::map<std::string, double> parameterVars;
    for (int i = 0; i < getProperty_variables().size(); ++i) {
        parameterVars[get_variables(i)] = getPropertyValue(i);
    }
    values[0] = m_program.evaluate(parameterVars);
}

void MocoExpressionBasedParameterGoal::printDescriptionImpl() const {
    log_info("        expression: {}", get_expression());
    for (int i = 0; i < getProperty_parameters().size(); ++i) {
        log_cout("        variable {}: {}", get_variables(i),
                 get_parameters(i).getName());
    }
}
