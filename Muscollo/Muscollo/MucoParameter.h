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

/// A MucoParameter allows you to optimize property values in an OpenSim Model.
/// To describe this parameter, you must provide the name of the property you
/// want to optimize, and the path to the component in the model where the
/// property exists. If the property is not a scalar, then you must also provide
/// the element index of the property you want to optimize. To optimize multiple
/// elements of a non-scalar property, use multiple MucoParameters.
/// By specifying multiple component paths, you can optimize the same property
/// in multiple components (each property will have the same value, as
/// determined by this parameter).
/// The following property types are currently supported:
///  - double
///  - Vec3
///  - Vec6
///
/// List properties are not currently supported.
///
/// The name you give to a MucoParameter does not need to match the
/// name of its model property.
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
///
/// The generic constructor can be used for more complex MucoParameter 
/// assignments. Here, we create a MucoParameter for the y-position of the mass
/// center of three different rigid bodies in the model:
/// @code
/// int propertyElt = 1; // y-position is the second element of the mass_center
/// std::vector<std::string> componentPaths = {"pelvis", "thigh", "shank"};
/// MucoParameter y_com("y_com", componentPaths, "mass_center", 
///         MucoBounds(-0.05, 0.05), propertyElt);
/// @endcode
class OSIMMUSCOLLO_API MucoParameter : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoParameter, Object);
public:
    // Default constructor.
    MucoParameter();
    /// This is a generic constructor that supports applying this parameter
    /// to multiple components and to non-scalar properties.
    /// Specifying propertyElt as -1 is treated as not specifying a property
    /// element.
    MucoParameter(const std::string& name,
        const std::vector<std::string>& componentPaths,
        const std::string& propertyName, const MucoBounds&,
        const int& propertyElt);
    // Simple parameter constructors.
    MucoParameter(const std::string& name, const std::string& componentPath,
        const std::string& propertyName, const MucoBounds&);
    MucoParameter(const std::string& name, const std::string& componentPath,
        const std::string& propertyName, const MucoBounds&, 
        const int& propertyElt);
    MucoParameter(const std::string& name,
        const std::vector<std::string>& componentPaths,
        const std::string& propertyName, const MucoBounds&);

    // Get and set methods.
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    MucoBounds getBounds() const
    {   return MucoBounds(getProperty_bounds()); }
    std::string getPropertyName() const
    {   return get_property_name(); }
    std::vector<std::string> getComponentPaths() const
    {  
       int numComponents = (int)getProperty_component_paths().size();
       std::vector<std::string> componentPaths(numComponents);
       for (int i = 0; i < numComponents; ++i) {
           componentPaths[i] = get_component_paths(i);
       }
       return componentPaths;
    }
    void setBounds(const MucoBounds& bounds)
    {   set_bounds(bounds.getAsArray()); }
    void setPropertyName(const std::string& propertyName)
    {   set_property_name(propertyName); }
    void appendComponentPath(const std::string& componentPath)
    {   append_component_paths(componentPath); }

    /// For use by solvers. This performs error checks and caches information
    /// about the model that is useful during the optimization.
    /// This method takes a non-const reference to the model because parameters
    /// need to be able to alter the model.
    void initialize(Model& model) const;
    /// Set the value of the model property to the passed-in parameter value.
    void applyParameterToModel(const double& value) const;

private:
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(bounds, double, 2,
        "1 value: required value over all time. "
        "2 values: lower, upper bounds on value.");
    OpenSim_DECLARE_LIST_PROPERTY(component_paths, std::string, "The path to "
        "the model component that owns the property associated with the "
        "MucoParameter.");
    OpenSim_DECLARE_PROPERTY(property_name, std::string, "The name of the "
        "model property associated with the MucoParameter.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(property_element, int, "For non-scalar "
        "model properties, the index of the element to be optimized.");

    mutable std::vector<SimTK::ReferencePtr<AbstractProperty>> m_property_refs;
    enum DataType {
        Type_double,
        Type_Vec3,
        Type_Vec6
    };
    mutable DataType m_data_type;
    void constructProperties();
    
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOPARAMETER_H