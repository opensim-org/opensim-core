#ifndef MOCO_MOCOINVERSE_H
#define MOCO_MOCOINVERSE_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoInverse.h                                                *
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

#include "Common/TableProcessor.h"
#include "MocoIterate.h"
#include "osimMocoDLL.h"

#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

class MocoInverse;

/// This class holds the solution from MocoInverseTool.
class MocoInverseSolution {
public:
    const MocoSolution& getMocoSolution() const { return m_mocoSolution; }

private:
    void setMocoSolution(MocoSolution mocoSolution) {
        m_mocoSolution = std::move(mocoSolution);
    }
    MocoSolution m_mocoSolution;
    friend class MocoInverse;
};

/// This tool solves problems in which the kinematics are prescribed and you
/// seek the actuator (e.g., muscle) behavior that may have given rise to the
/// provided kinematics. The term "inverse" describes methods that estimate
/// quantities from an observation; on the other hand, "forward" methods attempt
/// to predict (unobserved) behavior. In this case, "inverse" refers to the
/// multibody systems. This class can still be used to simulate muscles in a
/// "forward" or predictive sense.
///
/// The kinematics file must provide values for all coordinates (even those
/// labeled as dependent in a CoordinateCouplerConstraint); missing coordinates
/// are set to NaN.
///
/// The provided trajectory is altered to satisfy any enabled kinematic
/// constraints in the model. Filtering is performed before satisfying the
/// constraints.
///
/// @underdevelopment
class OSIMMOCO_API MocoInverse : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoInverse, Object);

public:
    OpenSim_DECLARE_OPTIONAL_PROPERTY(initial_time, double,
            "The start of the time interval in which to solve for muscle "
            "activity. "
            "All data must start at or before this time. "
            "(default: earliest time available in all provided data)");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(final_time, double,
            "The end of the time interval in which to solve for muscle "
            "activity. "
            "All data must end at or after this time. "
            "(default: latest time available in all provided data)");

    OpenSim_DECLARE_PROPERTY(mesh_interval, double,
            "The time duration of each mesh interval "
            "(default: 0.020 seconds).");

    OpenSim_DECLARE_PROPERTY(kinematics, TableProcessor,
            "Path to a STO file containing generalized coordinates "
            "to prescribe. The path can be absolute or relative to the setup "
            "file.");

    OpenSim_DECLARE_PROPERTY(external_loads_file, std::string,
            "XML file (.xml) containing the forces applied to the model as "
            "ExternalLoads.");

    OpenSim_DECLARE_PROPERTY(ignore_activation_dynamics, bool,
            "Ignore activation dynamics for all muscles in the model. "
            "If false, the muscle's setting is not modified."
            "(default: false).");

    OpenSim_DECLARE_PROPERTY(ignore_tendon_compliance, bool,
            "Ignore tendon_compliance for all muscles in the model. "
            "If false, the muscle's setting is not modified."
            "(default: false).");

    OpenSim_DECLARE_PROPERTY(create_reserve_actuators, double,
            "Create a reserve actuator (CoordinateActuator) for each "
            "unconstrained coordinate in the model, and add each to the model. "
            "Each actuator will have the specified `optimal_force`, which "
            "should be set low to "
            "discourage the use of the reserve actuators. (default is -1, "
            "which means no reserves are created)");

    MocoInverse() { constructProperties(); }

    void setModel(Model model) { m_model = std::move(model); }

    void setKinematics(TableProcessor kinematics) {
        set_kinematics(std::move(kinematics));
    }

    void setExternalLoadsFile(std::string fileName) {
        set_external_loads_file(std::move(fileName));
    }

    MocoInverseSolution solve() const;

private:
    void constructProperties();
    struct TimeInfo {
        double initialTime;
        double finalTime;
        int numMeshPoints;
    };
    TimeInfo calcInitialAndFinalTimes(
            // Time vector from a primary data source.
            const std::vector<double>& time0,
            // Time vector from a secondary data source.
            const std::vector<double>& time1, const double& meshInterval) const;

    // TODO: Move to property.
    Model m_model;
};

} // namespace OpenSim

#endif // MOCO_MOCOINVERSE_H
