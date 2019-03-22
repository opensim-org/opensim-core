#ifndef MOCO_POSITIONMOTION_H
#define MOCO_POSITIONMOTION_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: PositionMotion.h                                             *
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

#include "../osimMocoDLL.h"

#include <simbody/internal/Motion.h>

#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {

class Function;
class Coordinate;

class OSIMMOCO_API PositionMotion : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(PositionMotion, ModelComponent);

public:
    OpenSim_DECLARE_PROPERTY(default_enabled, bool,
            "Is this motion enabled by default? (default value: true)");
    OpenSim_DECLARE_PROPERTY(functions, FunctionSet,
            "Functions specifying the values of each coordinate.");

    PositionMotion() {
        constructProperty_default_enabled(true);
        constructProperty_functions(FunctionSet());
    }
    PositionMotion(std::string name) : PositionMotion() {
        setName(std::move(name));
    }
    ~PositionMotion() = default;
    void setPositionForCoordinate(
            const Coordinate& coord, const Function& position);
    /// This determines if, after Model::initSystem(), these prescribed
    /// motions are enabled or disabled.
    void setDefaultEnabled(bool enabled) { set_default_enabled(enabled); }
    bool getDefaultEnabled() const { return get_default_enabled(); }
    /// Use this after Model::initSystem() to set if the prescribed motions
    /// are enforced.
    void setEnabled(SimTK::State& state, bool enabled) const;
    bool getEnabled(const SimTK::State& state) const;
    /// Create a PositionMotion that prescribes kinematics for all coordinates
    /// in a model, given a data table containing coordinate values for all
    /// coordinates. If the table contains any columns that are not the names
    /// of coordinate value state variables, an exception is thrown (unless
    /// allowExtraColumns is true).
    ///
    /// @note If the data in the table violates kinematic constraints in the
    /// model, the resulting PositionMotion will also violate the kinematic
    /// constraints.
    ///
    /// @note This fuction ignores the inDegrees header metadata.
    static std::unique_ptr<PositionMotion> createFromTable(const Model& model,
            const TimeSeriesTable& coords, bool allowExtraColumns = false);

private:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendRealizeTopology(SimTK::State& state) const override;
    mutable SimTK::ResetOnCopy<std::vector<SimTK::Motion>> m_motions;
};

} // namespace OpenSim

#endif // MOCO_PRESCRIBEDPOSITIONMOTION_H
