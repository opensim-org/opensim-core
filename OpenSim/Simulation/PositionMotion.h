#ifndef OPENSIM_POSITIONMOTION_H
#define OPENSIM_POSITIONMOTION_H
/* -------------------------------------------------------------------------- *
 * OpenSim: PositionMotion.h                                                  *
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

#include <simbody/internal/Motion.h>

#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include "osimSimulationDLL.h"

namespace OpenSim {

class Function;
class Coordinate;
class StatesTrajectory;

/** This class prescribes the value, speed, and acceleration of all coordinates
in the model using SimTK::Motion. SimTK::Motion%s remove degrees of freedom
from the system rather than add constraints. This is an alternative to
prescribing kinematics using Coordinate's prescribed_function, which uses a
kinematic constraint. When prescribing motion, the system must compute
constraint forces to apply to enforce the prescribed motion;
such forces are available via SimbodyMatterSubsystem::findMotionForces().
@note This class requires that *all* coordinates are prescribed. */
class OSIMSIMULATION_API PositionMotion : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(PositionMotion, ModelComponent);

public:
    OpenSim_DECLARE_PROPERTY(default_enabled, bool,
            "Whether or not this motion is enabled by default (default value: "
            "true).");
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
    /// Set a function to calculate the position for a given coordinate.
    /// The speed and acceleration of the coordinate are obtained as derivatives
    /// of the provided function.
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
    /// coordinates using GCVSpline. If the table contains any columns that are
    /// not the names of coordinate value state variables, an exception is
    /// thrown (unless allowExtraColumns is true).
    ///
    /// @note If the data in the table violates kinematic constraints in the
    /// model, the resulting PositionMotion will also violate the kinematic
    /// constraints.
    ///
    /// @note This fuction ignores the inDegrees header metadata.
    static std::unique_ptr<PositionMotion> createFromTable(const Model& model,
            const TimeSeriesTable& coords, bool allowExtraColumns = false);
    /// Create a PositionMotion that prescribes kinematics for all coordinates
    /// in a model, given a StatesTrajectory.
    ///
    /// @note If the states trajectory violates kinematic constraints in the
    /// model, the resulting PositionMotion will also violate the kinematic
    /// constraints.
    static std::unique_ptr<PositionMotion> createFromStatesTrajectory(
            const Model& model, const StatesTrajectory& statesTraj);
    TimeSeriesTable exportToTable(const std::vector<double>& time) const;

private:
    /// Allocate SimTK::Motion%s.
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    /// Set the functions on the SimTK::Motions. We need to wait until Topology
    /// so that we can iterate through the system's MobilizedBodies.
    void extendRealizeTopology(SimTK::State& state) const override;
    mutable SimTK::ResetOnCopy<std::vector<SimTK::Motion>> m_motions;
};

} // namespace OpenSim

#endif // OPENSIM_PRESCRIBEDPOSITIONMOTION_H
