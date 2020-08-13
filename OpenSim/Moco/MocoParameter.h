#ifndef OPENSIM_MOCOPARAMETER_H
#define OPENSIM_MOCOPARAMETER_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoParameter.h                                                   *
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

#include "MocoBounds.h"

#include <OpenSim/Common/Property.h>
#include <OpenSim/Common/Object.h>
#include <SimTKcommon/internal/ReferencePtr.h>

namespace OpenSim {

class Model;

/** A MocoParameter allows you to optimize property values in an OpenSim Model.
To describe this parameter, you must provide the name of the property you
want to optimize and the path to the component in the model where the
property exists. If the property is not a scalar, then you must also provide
the element index of the property you want to optimize. To optimize multiple
elements of a non-scalar property, use multiple MocoParameters.
By specifying multiple component paths, you can optimize the same property
in multiple components (each property will have the same value, as
determined by this parameter).
The following property types are currently supported:
 - double
 - Vec3
 - Vec6

List properties are not currently supported.

The name you give to a MocoParameter does not need to match the
name of its model property.

The default initial guess for a parameter depends on the solver you use. Most
likely, the default initial guess is the midpoint of the bounds on the
parameter (NOT the value of the property in the model).

Create a MocoParameter from a property in the model:
@code
MocoParameter p0;
p0.setName("torso_mass");
p0.appendComponentPath("torso");
p0.setParameterName("mass");
MocoBounds massBounds(60, 80);
p0.setBounds(massBounds);
@endcode

Using the convenience constructor:
@code
MocoParameter p0("torso_mass", "torso", "mass", MocoBounds(60, 80));
@endcode

Here is a Matlab example of optimizing the optimal fiber length of a muscle:
@code
study = MocoStudy();
problem = study.updProblem();
param = MocoParameter('my_param_name', '/forceset/soleus_r', ...
        'optimal_fiber_length', MocoBounds(0.04, 0.06));
problem.addParameter(param);
@endcode

The generic constructor can be used for more complex MocoParameter
assignments. Here, we create a MocoParameter for the y-position of the mass
center of three different rigid bodies in the model:
@code
int propertyElt = 1; // y-position is the second element of the mass_center
std::vector<std::string> componentPaths = {
    "/bodyset/pelvis",
    "/bodyset/thigh",
    "/bodyset/shank"
};
MocoParameter y_com("y_com", componentPaths, "mass_center",
        MocoBounds(-0.05, 0.05), propertyElt);
@endcode
@par For developers
Every time the problem is solved, a copy of this parameter is used.
An individual instance of a parameter is only ever used in a single problem.
Therefore, there is no need to clear cache variables that you create in
initializeImpl(). Also, information stored in this parameter does not
persist across multiple solves. Lastly, a MocoParameter may be applied to
multiple models at once, as long as the value described in the MocoParameter
exists and initializeOnModel() is called on all models of interest. */
class OSIMMOCO_API MocoParameter : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoParameter, Object);
public:
    // Default constructor.
    MocoParameter();
    /// This is a generic constructor that supports applying this parameter
    /// to multiple components and to non-scalar properties.
    /// Specifying propertyElt as -1 is treated as not specifying a property
    /// element.
    MocoParameter(const std::string& name,
        const std::vector<std::string>& componentPaths,
        const std::string& propertyName, const MocoBounds&,
        const int& propertyElt);
    // Simple parameter constructors.
    MocoParameter(const std::string& name, const std::string& componentPath,
        const std::string& propertyName, const MocoBounds&);
    MocoParameter(const std::string& name, const std::string& componentPath,
        const std::string& propertyName, const MocoBounds&, 
        const int& propertyElt);
    MocoParameter(const std::string& name,
        const std::vector<std::string>& componentPaths,
        const std::string& propertyName, const MocoBounds&);

    // Get and set methods.
    /*** @details Note: the return value is constructed fresh on every call from
    the internal property. Avoid repeated calls to this function. */
    MocoBounds getBounds() const
    {   return get_MocoBounds(); }
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
    void setBounds(const MocoBounds& bounds)
    {   set_MocoBounds(bounds); }
    void setPropertyName(const std::string& propertyName)
    {   set_property_name(propertyName); }
    void appendComponentPath(const std::string& componentPath)
    {   append_component_paths(componentPath); }

    /** For use by solvers. This performs error checks and caches information
    about the model that is useful during the optimization.
    This method takes a non-const reference to the model because parameters
    need to be able to alter the model.
    If it is desired to apply this MocoParameter to multiple models, this
    should be called on all models of interest. The property references from
    each model will be appended to this MocoParameter's internal property
    reference list. */
    void initializeOnModel(Model& model) const;
    /** Set the value of the stored model properties, which may include
    properties from multiple models. */
    void applyParameterToModelProperties(const double& value) const;

    /** Print the name, property name, component paths, property element (if it
    exists), and bounds for this parameter. */
    void printDescription() const;

private:
    OpenSim_DECLARE_UNNAMED_PROPERTY(MocoBounds,
        "Bounds over all time.");
    OpenSim_DECLARE_LIST_PROPERTY(component_paths, std::string, "The path to "
        "the model component that owns the property associated with the "
        "MocoParameter.");
    OpenSim_DECLARE_PROPERTY(property_name, std::string, "The name of the "
        "model property associated with the MocoParameter.");
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

#endif // OPENSIM_MOCOPARAMETER_H
