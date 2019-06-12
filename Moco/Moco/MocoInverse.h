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
#include "MocoTool.h"
#include "osimMocoDLL.h"

#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

class MocoInverse;

/// This class holds the solution from MocoInverse.
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
class OSIMMOCO_API MocoInverse : public MocoTool {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoInverse, MocoTool);

public:

    OpenSim_DECLARE_PROPERTY(kinematics, TableProcessor,
            "Path to a STO file containing generalized coordinates "
            "to prescribe. The path can be absolute or relative to the setup "
            "file.");

    MocoInverse() { constructProperties(); }

    void setKinematics(TableProcessor kinematics) {
        set_kinematics(std::move(kinematics));
    }

    MocoInverseSolution solve() const;

private:
    void constructProperties();
};

} // namespace OpenSim

#endif // MOCO_MOCOINVERSE_H
