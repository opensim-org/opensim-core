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

/** TODO */
class OSIMMOCO_API MocoScaleFactor : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoScaleFactor, Component);

public:
    MocoScaleFactor();
    MocoScaleFactor(const std::string& name, const std::string& path,
            const MocoBounds&);

    double getScaleFactor() const { return get_scale_factor(); }
    void setScaleFactor(double value) { set_scale_factor(value); }

    const std::string& getComponentPath() const { return get_component_path(); }
    void setComponentPath(const std::string& path) { set_component_path(path); }

    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    MocoBounds getBounds() const { return MocoBounds(getProperty_bounds()); }
    void setBounds(const MocoBounds& bounds) {
        set_bounds(bounds.getAsArray());
    }
protected:
    OpenSim_DECLARE_PROPERTY(scale_factor, double, "TODO");
    OpenSim_DECLARE_PROPERTY(component_path, std::string, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(bounds, double, 2,
        "1 value: required value over all time. "
        "2 values: lower, upper bounds on value over all time.");
private:
    void constructProperties();
};

} // namespace OpenSim

#endif //OPENSIM_MOCOSCALEFACTOR_H
