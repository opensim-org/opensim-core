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

#include "MocoIterate.h"
#include "osimMocoDLL.h"

#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

class MocoInverse;

class MocoInverseSolution {
public:
    const MocoSolution& getMocoSolution() const;

private:
    void setMocoSolution(MocoSolution mocoSolution) {
        m_mocoSolution = std::move(mocoSolution);
    }
    MocoSolution m_mocoSolution;
    friend class MocoInverse;
};

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

    OpenSim_DECLARE_PROPERTY(mesh_point_frequency, int,
            "The number of mesh points per second of motion "
            "(default: 50 mesh points / second).");

    OpenSim_DECLARE_PROPERTY(external_loads_file, std::string, "TODO");

    OpenSim_DECLARE_PROPERTY(create_reserve_actuators, double,
            "Create a reserve actuator (CoordinateActuator) for each "
            "unconstrained coordinate in the model, and add each to the model. "
            "Each actuator will have the specified `optimal_force`, which "
            "should be set low to "
            "discourage the use of the reserve actuators. (default is -1, "
            "which means no reserves are created)");

    MocoInverse() {
        constructProperties();
    }

    void setModel(Model model) { m_model = std::move(model); }

    void setKinematicsFile(std::string fileName) {
        m_kinematicsFileName = std::move(fileName);
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
            const std::vector<double>& time1,
            const int& meshPointFrequency) const;

    Model m_model;
    std::string m_kinematicsFileName;
};

} // namespace OpenSim

#endif // MOCO_MOCOINVERSE_H
