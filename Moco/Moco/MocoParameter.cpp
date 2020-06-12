/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoParameter.cpp                                            *
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

#include "MocoParameter.h"
#include "MocoUtilities.h"
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

MocoParameter::MocoParameter() {
    constructProperties();
    if (getName().empty()) setName("parameter");
}

MocoParameter::MocoParameter(const std::string& name,
    const std::vector<std::string>& componentPaths,
    const std::string& propertyName,
    const MocoBounds& bounds,
    const int& propertyElt) : MocoParameter() {
    setName(name);
    set_MocoBounds(bounds);
    Array<std::string> compPaths;
    for (auto& str : componentPaths) {
        compPaths.append(str);
    }
    set_component_paths(compPaths);
    set_property_name(propertyName);
    // Internally, the constructors use -1 to signal that propertyElt was not
    // provided.
    if (propertyElt != -1)
        set_property_element(propertyElt);
}

MocoParameter::MocoParameter(const std::string& name,
    const std::string& componentPath,
    const std::string& propertyName,
    const MocoBounds& bounds) : 
    MocoParameter(name, componentPath, propertyName, bounds, -1) {}

MocoParameter::MocoParameter(const std::string& name,
    const std::string& componentPath,
    const std::string& propertyName,
    const MocoBounds& bounds,
    const int& propertyElt) : 
    MocoParameter(name, std::vector<std::string>{componentPath}, 
        propertyName, bounds, propertyElt) {}

MocoParameter::MocoParameter(const std::string& name,
    const std::vector<std::string>& componentPaths,
    const std::string& propertyName,
    const MocoBounds& bounds) :
    MocoParameter(name, componentPaths, propertyName, bounds, -1) {}

void MocoParameter::constructProperties() {
    constructProperty_MocoBounds(MocoBounds());
    constructProperty_component_paths();
    constructProperty_property_name("");
    constructProperty_property_element();
}

void MocoParameter::initializeOnModel(Model& model) const {
    
    OPENSIM_THROW_IF_FRMOBJ(getProperty_component_paths().empty(), Exception,
        "A model component name must be provided.");
    OPENSIM_THROW_IF_FRMOBJ(get_property_name().empty(), Exception,
        "A component property name must be provided.");

    for (int i = 0; i < (int)getProperty_component_paths().size(); ++i) {
        // Get model component.
        auto& component = model.updComponent(get_component_paths(i));
        // Get component property.
        auto* ap = &component.updPropertyByName(get_property_name());
        OPENSIM_THROW_IF_FRMOBJ(ap->isListProperty(), Exception, 
            "MocoParameter does not support list properties.");

        // Type detection and property element value error checking.
        if (dynamic_cast<Property<double>*>(ap)) {
            OPENSIM_THROW_IF_FRMOBJ(!getProperty_property_element().empty(),
                Exception,
                "A property element index was specified for "
                "a scalar model property. Check if model property was intended "
                "to be non-scalar or if property element was provided by "
                "mistake (e.g. wrong constructor used).");
            m_data_type = Type_double;
        } else {
            OPENSIM_THROW_IF_FRMOBJ(getProperty_property_element().empty(),
                Exception, "Must specify a property element for "
                "non-scalar propeties.");
            OPENSIM_THROW_IF_FRMOBJ(get_property_element() < 0, Exception,
                    "Expected property element to be non-negative, but {} was "
                    "provided.",
                    get_property_element());
            if (dynamic_cast<Property<SimTK::Vec3>*>(ap)) {
                OPENSIM_THROW_IF_FRMOBJ(get_property_element() > 2, Exception,
                        "The property element for a Vec3 property must be "
                        "between 0 and 2, but the value {} was provided.",
                        get_property_element());
                m_data_type = Type_Vec3;
            }
            else if (dynamic_cast<Property<SimTK::Vec6>*>(ap)) {
                OPENSIM_THROW_IF_FRMOBJ(get_property_element() > 5, Exception,
                        "The property element for a Vec6 property must be "
                        "between 0 and 5, but the value {} was provided.",
                        get_property_element());
                m_data_type = Type_Vec6;
            }
            else {
                OPENSIM_THROW_FRMOBJ(Exception,
                    "Data type of specified model property not supported.");
            }
        }

        m_property_refs.emplace_back(ap);
    }
}

void MocoParameter::printDescription() const {
    const std::vector<std::string> componentPaths = getComponentPaths();

    std::string propertyElementStr;
    if (getProperty_property_element().empty()) {
        propertyElementStr = "n/a";
    } else {
        propertyElementStr =
                fmt::format("{}", getProperty_property_element().getValue());
    }

    log_cout("  {}. model property name: {}. component paths: {}. "
             "property element: {}. bounds: {}",
            getName(), getPropertyName(), fmt::join(componentPaths, ", "),
            propertyElementStr, getBounds());
}

void MocoParameter::applyParameterToModelProperties(const double& value) const {
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
