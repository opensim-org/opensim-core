/* -------------------------------------------------------------------------- *
* OpenSim: MocoExpressionBasedParameterGoal.h                                 *
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
#include <lepton/Parser.h>
#include <lepton/ParsedExpression.h>

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

void MocoExpressionBasedParameterGoal::constructProperties() {
    constructProperty_expression("");
    constructProperty_parameters();
    constructProperty_variable_names();
}

void MocoExpressionBasedParameterGoal::initializeOnModelImpl(const Model& model) const {
    m_parameterProg = Lepton::Parser::parse(get_expression()).optimize().createProgram();
    setRequirements(1, 1);

    // store parameter property ref
    for (int i = 0; i < getProperty_parameters().size(); i++) {
        std::string componentPath = get_parameters(i).getComponentPaths()[0];  // first one bc they should all be the same
        const auto& component = model.getComponent(componentPath);
        // Get component property.
        const auto* ap = &component.getPropertyByName(get_parameters(i).getPropertyName());
        m_property_refs.emplace_back(ap);

        // get the type of the property, and element
        OPENSIM_THROW_IF_FRMOBJ(ap->isListProperty(), Exception,
            "MocoParameter does not support list properties.");

        // Type detection and property element value error checking.
        if (dynamic_cast<const Property<double>*>(ap)) {
            m_data_types.emplace_back(Type_double);
        } else {
            if (dynamic_cast<const Property<SimTK::Vec3>*>(ap)) {
                m_data_types.emplace_back(Type_Vec3);
                m_indices.emplace_back(get_parameters(i).getElement());
            }
            else if (dynamic_cast<const Property<SimTK::Vec6>*>(ap)) {
                m_data_types.emplace_back(Type_Vec6);
                m_indices.emplace_back(get_parameters(i).getElement());
            }
            else {
                OPENSIM_THROW_FRMOBJ(Exception,
                    "Data type of specified model property not supported.");
            }
        }

        m_property_refs.emplace_back(ap);
    }
}

double MocoExpressionBasedParameterGoal::getPropertyValue(int i) const {
    OPENSIM_THROW_IF_FRMOBJ(m_property_refs.size() < 1, Exception,
            "No properties to get value of.")
    const auto& propRef = m_property_refs[i];

    if (m_data_types[i] == Type_double) {
        return static_cast<const Property<double>*>(propRef.get())->getValue();
    }

    int elt = m_indices[i];
    if (m_data_types[i] == Type_Vec3) {
        return static_cast<const Property<SimTK::Vec3>*>(propRef.get())->getValue()[elt];
    }

    if (m_data_types[i] == Type_Vec6) {
        return static_cast<const Property<SimTK::Vec6>*>(propRef.get())->getValue()[elt];
    }

    OPENSIM_THROW_FRMOBJ(Exception, "Properties not of a recognized type.");
}

void MocoExpressionBasedParameterGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {

    std::map<std::string, double> parameterVars;
    for (int i=0; i<getProperty_variable_names().size(); ++i) {
        std::string variableName = getProperty_variable_names()[i];
        MocoParameter parameter = get_parameters(i);
        parameterVars[variableName] = getPropertyValue(i);
    }
    integrand = m_parameterProg.evaluate(parameterVars);
}

void MocoExpressionBasedParameterGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& values) const {
    values[0] = input.integral;
}

