#ifndef MOCO_MOCOVARIABLEINFO_H
#define MOCO_MOCOVARIABLEINFO_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoProblem.h                                                *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia, Nicholas Bianco                             *
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

namespace OpenSim {

/// Bounds on continuous variables (states, multipliers, etc). For states, the 
/// name should correspond to path of a state variable in the model.
class OSIMMOCO_API MocoVariableInfo : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoVariableInfo, Object);
public:
    MocoVariableInfo();
    MocoVariableInfo(const std::string& name, const MocoBounds&,
            const MocoInitialBounds&, const MocoFinalBounds&);

    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    MocoBounds getBounds() const
    {   return MocoBounds(getProperty_bounds()); }
    /// @copydoc getBounds()
    MocoInitialBounds getInitialBounds() const
    {   return MocoInitialBounds(getProperty_initial_bounds()); }
    /// @copydoc getBounds()
    MocoFinalBounds getFinalBounds() const
    {   return MocoFinalBounds(getProperty_final_bounds()); }

    /// Throws an exception if initial and final bounds are not within bounds.
    // TODO Move to finalizeFromProperties() and cache MocoBounds.
    void validate() const;

    /// Print the bounds on this variable.
    void printDescription(std::ostream& stream = std::cout) const;

protected:
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(bounds, double, 2,
            "1 value: required value over all time. "
            "2 values: lower, upper bounds on value over all time.");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(initial_bounds, double, 2,
            "1 value: required initial value. "
            "2 values: lower, upper bounds on initial value.");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(final_bounds, double, 2,
            "1 value: required final value. "
            "2 values: lower, upper bounds on final value.");

private:
    void constructProperties();

};

/// Bounds and actuator information for continuous control variables. 
// TODO remove this class when we support named controls in OpenSim
class OSIMMOCO_API MocoControlInfo : public MocoVariableInfo {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoControlInfo, MocoVariableInfo);
public:
    MocoControlInfo() { constructProperties(); }
    MocoControlInfo(const std::string& actuatorName, const MocoBounds&,
        const MocoInitialBounds&, const MocoFinalBounds&);
    MocoControlInfo(const std::string& actuatorName, int index,
        const MocoBounds&, const MocoInitialBounds&, const MocoFinalBounds&);

    const std::string& getActuatorName() const 
    {   return get_actuator_name(); }
    int getControlIndex() const 
    {   return get_control_index(); }

protected:
    OpenSim_DECLARE_PROPERTY(actuator_name, std::string,
        "The name of the actuator associated with this control variable.");
    OpenSim_DECLARE_PROPERTY(control_index, int,
        "The index to this control variable in the associated actuator's "
        "control vector.");

private:
    void constructProperties();

};

} // namespace OpenSim

#endif // MOCO_MOCOVARIABLEINFO_H
