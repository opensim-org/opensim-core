#ifndef MUSCOLLO_MUCOPARAMETER_H
#define MUSCOLLO_MUCOPARAMETER_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoParameter.h                                          *
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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {


/// TODO: docs
class OSIMMUSCOLLO_API MucoParameter : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoParameter, Object);
public:
    // Default constructor.
    MucoParameter();
    // Simple parameter constructor.
    MucoParameter(const std::string& name, const std::string& modelProperty,
        const std::string& modelComponent, const MucoBounds&);
    // Generic parameter constructor.
    //MucoParameter(const std::string& name, const std::string& modelProperty, 
    //        const std::vector<std::string>& modelComponents,
    //        const MucoBounds&, const unsigned& propertyElement = 0);

    // Get and set methods.
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    MucoBounds getBounds() const
    {   return MucoBounds(getProperty_bounds()); }
    std::string getModelProperty() const
    {   return get_model_property(); }
    std::string getModelComponent() const
    {   return get_model_component(); }
    void setBounds(const MucoBounds& bounds)
    {   set_bounds(bounds.getAsArray()); }
    void setModelProperty(const std::string& modelProperty)
    {   set_model_property(modelProperty); }
    void setModelComponent(const std::string& modelComponent)
    {   set_model_component(modelComponent); }

    /// For use by solvers. This also performs error checks on the Problem.
    void initialize(const Model& model) const {
        OPENSIM_THROW_IF(get_model_component().empty(), Exception,
            "TODO: must set component name");
        OPENSIM_THROW_IF(get_model_property().empty(), Exception,
            "TODO: must set property name");

        m_property.reset(&model.getComponent(
            get_model_component()).getPropertyByName(get_model_property()));
    }

private:
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(bounds, double, 2,
        "1 value: required value over all time. "
        "2 values: lower, upper bounds on value over all time.");
    OpenSim_DECLARE_PROPERTY(model_property, std::string, "TODO");
    // TODO: make this a list property (model_components)
    OpenSim_DECLARE_PROPERTY(model_component, std::string, "TODO");

    mutable SimTK::ReferencePtr<const AbstractProperty> m_property;
    void constructProperties();
};


} // namespace OpenSim

#endif // MUSCOLLO_MUCOPROBLEM_H