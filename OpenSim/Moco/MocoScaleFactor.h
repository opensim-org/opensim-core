#ifndef OPENSIM_MOCOSCALEFACTOR_H
#define OPENSIM_MOCOSCALEFACTOR_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoScaleFactor.h                                                 *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2021 Stanford University and the Authors                     *
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

#include <OpenSim/Common/Component.h>
#include "MocoBounds.h"

namespace OpenSim {

/** A scale factor is a scalar value optimized by Moco that can be used to scale
a tracked reference quantity in the cost function. Scale factors can be added to
a MocoProblem via MocoGoals, and this class enables the use of MocoParameter
to optimize scale factors values by providing the 'scale_factor' property.
MocoScaleFactor derives from Component, since it must be appended to the model
internal to MocoProblem as MocoParameter can only optimize model parameter values
contained in properties. Users do not need to interact with this class directly,
but rather use the 'addScaleFactor()' interface provided by MocoGoals that
support scale factors. How the scale factor is utilized in the cost function is
specific to each MocoGoal. */
class OSIMMOCO_API MocoScaleFactor : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoScaleFactor, Component);

public:
    MocoScaleFactor();
    MocoScaleFactor(const std::string& name, const MocoBounds&);

    double getScaleFactor() const { return get_scale_factor(); }
    void setScaleFactor(double value) { set_scale_factor(value); }

    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    MocoBounds getBounds() const { return MocoBounds(getProperty_bounds()); }
    void setBounds(const MocoBounds& bounds) {
        set_bounds(bounds.getAsArray());
    }
protected:
    // Properties
    OpenSim_DECLARE_PROPERTY(scale_factor, double, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(bounds, double, 2,
        "1 value: required value over all time. "
        "2 values: lower, upper bounds on value over all time.");
private:
    void constructProperties();
};

} // namespace OpenSim

#endif //OPENSIM_MOCOSCALEFACTOR_H
