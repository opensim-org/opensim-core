#ifndef OPENSIM_MOCOTOOL_H
#define OPENSIM_MOCOTOOL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoTool.h                                                        *
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

#include "OpenSim/Actuators/ModelProcessor.h"

#include <OpenSim/Common/Object.h>

namespace OpenSim {

/** This is a base class for solving problems that depend on an observed motion
using Moco's optimal control methods.
MocoTool Properties
===================
Mesh interval
-------------
A smaller mesh interval increases the convergence time, but is necessary
for fast motions or problems with stiff differential equations (e.g.,
stiff tendons).
For gait, consider using a mesh interval between 0.01 and 0.05 seconds.
Try solving your problem with decreasing mesh intervals and choose a mesh
interval at which the solution stops changing noticeably.

Reserve actuators
-----------------
Sometimes it is not possible to achieve the desired motion using
muscles alone. There are multiple possible causes for this:
  - the muscles are not strong enough to achieve the required
    net joint moments,
  - the net joint moments change more rapidly than activation and
    deactivation time constants allow,
  - the filtering of the data causes unrealistic desired net joint moments.
You may want to add "reserve" actuators to your model.
This can be done with the ModOpAddReserves model operator. */
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

    OpenSim_DECLARE_PROPERTY(clip_time_range, bool,
            "Set the time range to be 1e-3 shorter on both ends to leave space "
            "for finite difference estimates (default: false).");

    OpenSim_DECLARE_PROPERTY(
            model, ModelProcessor, "The musculoskeletal model to use.");

    MocoTool() { constructProperties(); }

    void setModel(ModelProcessor model) { set_model(std::move(model)); }

protected:
#ifndef SWIG
    struct TimeInfo {
        double initial = -SimTK::Infinity;
        double final = SimTK::Infinity;
        int numMeshIntervals = -1;
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

    /// Get the canonicalized absolute pathname with respect to the setup file
    /// directory from a given pathname which can be relative or absolute. Here,
    /// canonicalized means that the pathname is analyzed and possibly modified
    /// to conform to the current platform.
    std::string getFilePath(const std::string& file) const;

    /// Get the (canonicalized) absolute directory containing the file from
    /// which this tool was loaded. If the tool was not loaded from a file, this
    /// returns an empty string.
    std::string getDocumentDirectory() const;

#endif
private:
    void constructProperties();
};

} // namespace OpenSim

#endif // OPENSIM_MOCOTOOL_H
