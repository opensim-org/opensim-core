#ifndef OPENSIM_MOCOMARKERFINALGOAL_H
#define OPENSIM_MOCOMARKERFINALGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoMarkerFinalGoal.h                                             *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include "MocoGoal.h"

namespace OpenSim {

class Point;

/** The squared distance between a single model point location and reference
location in the final state.
@ingroup mocogoal */
class OSIMMOCO_API MocoMarkerFinalGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoMarkerFinalGoal, MocoGoal);
public:
    MocoMarkerFinalGoal() { constructProperties(); }
    MocoMarkerFinalGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoMarkerFinalGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    /// The name of a Point (e.g., Marker, Station) in the model whose
    /// final location should match the reference.
    void setPointName(std::string pointName) {
        set_point_name(std::move(pointName));
    }

    /// Set the desired final location of the point, expressed in ground.
    void setReferenceLocation(SimTK::Vec3 refLocationInGround) {
        set_reference_location(std::move(refLocationInGround));
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;
    void printDescriptionImpl() const override;
private:

    OpenSim_DECLARE_PROPERTY(point_name, std::string,
        "The name (path) of a Point (Marker, Station) component in the model "
        "whose location should match the reference.");
    OpenSim_DECLARE_PROPERTY(reference_location, SimTK::Vec3,
        "The desired final location of the point, expressed in ground.");

    void constructProperties();

    mutable SimTK::ReferencePtr<const Point> m_point;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOMARKERFINALGOAL_H
