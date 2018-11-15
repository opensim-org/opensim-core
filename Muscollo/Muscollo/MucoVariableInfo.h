#ifndef MUSCOLLO_MUCOVARIABLEINFO_H
#define MUSCOLLO_MUCOVARIABLEINFO_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoProblem.h                                            *
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

#include "MucoBounds.h"

namespace OpenSim {

/// Bounds on continuous variables (states, controls). The name should
/// correspond to path of a state variable or an actuator in the model.
class OSIMMUSCOLLO_API MucoVariableInfo : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoVariableInfo, Object);
public:
    MucoVariableInfo();
    MucoVariableInfo(const std::string& name, const MucoBounds&,
            const MucoInitialBounds&, const MucoFinalBounds&);

    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    MucoBounds getBounds() const
    {   return MucoBounds(getProperty_bounds()); }
    /// @copydoc getBounds()
    MucoInitialBounds getInitialBounds() const
    {   return MucoInitialBounds(getProperty_initial_bounds()); }
    /// @copydoc getBounds()
    MucoFinalBounds getFinalBounds() const
    {   return MucoFinalBounds(getProperty_final_bounds()); }

    /// Throws an exception if initial and final bounds are not within bounds.
    // TODO Move to finalizeFromProperties() and cache MucoBounds.
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

} // namespace OpenSim

#endif // MUSCOLLO_MUCOVARIABLEINFO_H
