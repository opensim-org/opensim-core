#ifndef MOCO_MOCOTOOL_H
#define MOCO_MOCOTOOL_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoTool.h                                                   *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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

#include "ModelProcessor.h"

#include <OpenSim/Common/Object.h>

namespace OpenSim {

/// This is a base class for solving problems that depend on an observed motion
/// using Moco's optimal control methods.
class MocoTool : public Object {
    OpenSim_DECLARE_ABSTRACT_OBJECT(MocoTool, Object);

public:
    OpenSim_DECLARE_OPTIONAL_PROPERTY(initial_time, double,
            "The start of the time interval. "
            "All data must start at or before this time. "
            "(default: earliest time available in all provided data)");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(final_time, double,
            "The end of the time interval. "
            "All data must end at or after this time. "
            "(default: latest time available in all provided data)");

    OpenSim_DECLARE_PROPERTY(mesh_interval, double,
            "The time duration of each mesh interval "
            "(default: 0.020 seconds).");

    OpenSim_DECLARE_PROPERTY(
            model, ModelProcessor, "The musculoskeletal model to use.");

    MocoTool() { constructProperties(); }

    void setModel(ModelProcessor model) { set_model(std::move(model)); }

protected:
    struct TimeInfo {
        double initial = -SimTK::Infinity;
        double final = SimTK::Infinity;
        int numMeshPoints = -1;
    };
    /// This function updates a TimeInfo so the initial and final times are
    /// within the data times provided. If the user provided a value for the
    /// initial_time or final_time properties, then this ensures the
    /// user-provided times are within the data times, and the info is updated
    /// to use the user-provided times.
    /// Finally, the TimeInfo.numMeshPoints field is updated based on the
    /// mesh_interval property.
    void updateTimeInfo(const std::string& dataLabel, const double& dataInitial,
            const double& dataFinal, TimeInfo& info) const;

private:
    void constructProperties();
};

} // namespace OpenSim

#endif // MOCO_MOCOTOOL_H
