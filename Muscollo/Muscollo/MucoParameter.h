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

#include "MucoBounds.h"

#include <OpenSim/Common/Property.h>
#include <OpenSim/Common/Object.h>
#include <SimTKcommon/internal/ReferencePtr.h>

namespace OpenSim {

class Model;

/// The bounds and model information (component and property) for a parameter
/// to be optimized in the model. The parameter name does not need to match the 
/// name of the model property.
///
/// Create a MucoParameter from a property in the model:
/// @code
/// MucoParameter p0;
/// p0.setName("torso_mass");
/// p0.setComponentPath("torso");
/// p0.setParameterName("mass");
/// MucoBounds massBounds(60, 80);
/// p0.setBounds(massBounds);
/// @endcode
///
/// Using the convenience constructor:
/// @code
/// MucoParameter p0("torso_mass", "torso", "mass", MucoBounds(60, 80));
/// @endcode
// TODO: generic constructor example when implemented
class OSIMMUSCOLLO_API MucoParameter : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoParameter, Object);
public:
    // Default constructor.
    MucoParameter();
    // Simple parameter constructor.
    MucoParameter(const std::string& name, const std::string& componentPath,
        const std::string& propertyName, const MucoBounds&);
    // TODO: Generic parameter constructor.
    //MucoParameter(const std::string& name, 
    //  const std::vector<std::string>& componentPaths,
    //  const std::string& propertyName, const MucoBounds&, 
    //  const unsigned& propertyElt = 0);

    // Get and set methods.
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    MucoBounds getBounds() const
    {   return MucoBounds(getProperty_bounds()); }
    std::string getPropertyName() const
    {   return get_property_name(); }
    std::string getComponentPath() const
    {   return get_component_path(); }
    void setBounds(const MucoBounds& bounds)
    {   set_bounds(bounds.getAsArray()); }
    void setPropertyName(const std::string& propertyName)
    {   set_property_name(propertyName); }
    void setComponentPath(const std::string& componentPath)
    {   set_component_path(componentPath); }

    /// For use by solvers. This also performs error checks.
    void initialize(const Model& model) const;
    /// Set the value of the model property to the passed-in parameter value.
    void applyParameterToModel(const double& value) const;

private:
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(bounds, double, 2,
        "1 value: required value over all time. "
        "2 values: lower, upper bounds on value.");
    // TODO: support multiple components (i.e. make this a list property)
    OpenSim_DECLARE_PROPERTY(component_path, std::string, "The path to the "
        "model component that owns the property associated with the "
        "MucoParameter.");
    OpenSim_DECLARE_PROPERTY(property_name, std::string, "The name of the "
        "model property associated with the MucoParameter.");

    mutable SimTK::ReferencePtr<Property<double>> m_property;
    void constructProperties();
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOPARAMETER_H