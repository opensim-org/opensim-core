/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoParameter.cpp                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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

#include "MucoParameter.h"
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

MucoParameter::MucoParameter() {
    constructProperties();
    if (getName().empty()) setName("parameter");
}

MucoParameter::MucoParameter(const std::string& name,
    const std::vector<std::string>& componentPaths,
    const std::string& propertyName,
    const MucoBounds& bounds,
    const int& propertyElt) : MucoParameter() {
    setName(name);
    set_bounds(bounds.getAsArray());
    Array<std::string> compPaths;
    for (auto& str : componentPaths) {
        compPaths.append(str);
    }
    set_component_paths(compPaths);
    set_property_name(propertyName);
    set_property_element(propertyElt);
}

MucoParameter::MucoParameter(const std::string& name,
    const std::string& componentPath,
    const std::string& propertyName,
    const MucoBounds& bounds) : 
    MucoParameter(name, componentPath, propertyName, bounds, -1) {}

MucoParameter::MucoParameter(const std::string& name,
    const std::string& componentPath,
    const std::string& propertyName,
    const MucoBounds& bounds,
    const int& propertyElt) : 
    MucoParameter(name, std::vector<std::string>{componentPath}, 
        propertyName, bounds, propertyElt) {}

MucoParameter::MucoParameter(const std::string& name,
    const std::vector<std::string>& componentPaths,
    const std::string& propertyName,
    const MucoBounds& bounds) :
    MucoParameter(name, componentPaths, propertyName, bounds, -1) {}

void MucoParameter::constructProperties() {
    constructProperty_bounds();
    constructProperty_component_paths();
    constructProperty_property_name("");
    constructProperty_property_element();
}

void MucoParameter::initialize(const Model& model) const {
    
    OPENSIM_THROW_IF_FRMOBJ(getProperty_component_paths().empty(), Exception,
        "A model component name must be provided.");
    OPENSIM_THROW_IF_FRMOBJ(get_property_name().empty(), Exception,
        "A component property name must be provided.");
    OPENSIM_THROW_IF_FRMOBJ(getProperty_property_element().empty(), Exception,
        "A value for the property element must be specified: either -1 if a "
        "scalar property or a valid index if a vector property.");

    for (int i = 0; i < (int)getProperty_component_paths().size(); ++i) {
        // Get model component.
        auto& component = model.getComponent(get_component_paths(i));
        // Get component property.
        auto* ap  = const_cast<AbstractProperty*>(&component.getPropertyByName(
            get_property_name()));
        OPENSIM_THROW_IF_FRMOBJ(ap->isListProperty(), Exception, 
            "A MucoParameter cannot be a list property.");

        // Type detection and property element value error checking.
        if (auto* p = dynamic_cast<Property<double>*>(ap)) {
            OPENSIM_THROW_IF(get_property_element() >= 0, Exception, 
                "A non-negative property element index was specified for "
                "a scalar model property. Check if model property was intended "
                "to be non-scalar or if property element was provided by "
                "mistake (e.g. wrong constructor used).");
            m_data_type = Type_double;
        } else if (auto* p = dynamic_cast<Property<SimTK::Vec3>*>(ap)) {
            OPENSIM_THROW_IF((get_property_element() < 0) ||
                (get_property_element() > 2), Exception, "The property element "
                "for a Vec3 property must be between 0 and 2, but the value "
                + std::to_string(get_property_element()) + "was provided.");
            m_data_type = Type_Vec3;
        } else if (auto* p = dynamic_cast<Property<SimTK::Vec6>*>(ap)) {
            OPENSIM_THROW_IF((get_property_element() < 0) ||
                (get_property_element() > 5), Exception, "The property element "
                "for a Vec6 property must be between 0 and 5, but the value "
                + std::to_string(get_property_element()) + "was provided.");
            m_data_type = Type_Vec6;
        } else { 
            OPENSIM_THROW(Exception, "Data type of specified model property "
                "not supported."); 
        }

        m_property_refs.emplace_back(ap);
    }
}

void MucoParameter::applyParameterToModel(const double& value) const {
    for (auto& propRef : m_property_refs) {

        if (m_data_type == Type_double) {
            static_cast<Property<double>*>(propRef.get())->setValue(value);
        } else {
            int elt = get_property_element();
            if (m_data_type == Type_Vec3) {
                static_cast<Property<SimTK::Vec3>*>(
                    propRef.get())->updValue()[elt] = value;
            } else if (m_data_type == Type_Vec6) {
                static_cast<Property<SimTK::Vec6>*>(
                    propRef.get())->updValue()[elt] = value;
            }
        }
    }
}