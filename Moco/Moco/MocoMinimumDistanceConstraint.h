#ifndef MOCO_MOCOMINIMUMDISTANCECONSTRAINT_H
#define MOCO_MOCOMINIMUMDISTANCECONSTRAINT_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoMinimumDistanceConstraint.h                              *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

#include "MocoConstraint.h"
#include "osimMocoDLL.h"

namespace OpenSim {

class OSIMMOCO_API MocoMinimumDistanceConstraintPair : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoMinimumDistanceConstraintPair, Object);

public:
    OpenSim_DECLARE_PROPERTY(first_frame_path, std::string,
            "The first model frame path of the pair.");
    OpenSim_DECLARE_PROPERTY(second_frame_path, std::string,
            "The second model frame path of the pair.");
    OpenSim_DECLARE_PROPERTY(minimum_distance, double,
            "The minimum distance apart that the two frame origins must be.");

    MocoMinimumDistanceConstraintPair();
    MocoMinimumDistanceConstraintPair(std::string firstFramePath,
            std::string secondFramePath, double minimum_distance);

private:
    void constructProperties();
};

/// This goal enforces that a minimum distance is kept between the origins of 
/// pairs of model frames. Frame pairs, and the minimum distance they are to be 
/// kept apart, are specified via a MocoMinimumDistancConstraintPair. Distance
/// is computed by taking the norm of the relative position vector in ground 
/// between model frame origins. Any model component derived from Frame is valid 
/// to be included in a frame pair, and any number of frame pairs may be append 
/// to this constraint via addFramePair().
/// 
/// This constraint can be used as a simple method for preventing bodies in your
/// model from intersecting during an optimization. For example, the
/// following prevents feet from intersecting during a walking optimization:
/// @code
/// distance = problem.addPathConstraint<MocoMinimumDistanceConstraint>();
/// distance.setName("minimum_distance"):
/// distance.addFramePair({'/bodyset/calcn_l', '/bodyset/calcn_r', 0.1});
/// distance.addFramePair({'/bodyset/toes_l', '/bodyset/toes_r', 0.1});
/// distance.addFramePair({'/bodyset/calcn_l', '/bodyset/toes_r', 0.1});
/// distance.addFramePair({'/bodyset/toes_l', '/bodyset/calcn_r', 0.1});
/// @endcode
class OSIMMOCO_API MocoMinimumDistanceConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoMinimumDistanceConstraint, MocoPathConstraint);

public:
    MocoMinimumDistanceConstraint();

    /// Add a MocoMinimumDistanceConstraintPair 
    void addFramePair(MocoMinimumDistanceConstraintPair pair) {
        append_frame_pairs(std::move(pair));
    }

protected:
    void initializeOnModelImpl(
            const Model& model, const MocoProblemInfo&) const override;
    void calcPathConstraintErrorsImpl(
            const SimTK::State& state, SimTK::Vector& errors) const override;

private:
    OpenSim_DECLARE_LIST_PROPERTY(frame_pairs, 
            MocoMinimumDistanceConstraintPair, 
            "Pairs of frames whose origins are constrained to be a minimum "
            "distance apart.");

    void constructProperties();
    mutable std::vector<std::pair<SimTK::ReferencePtr<const Frame>,
            SimTK::ReferencePtr<const Frame>>> m_frame_pairs;
};

} // namespace OpenSim

#endif // MOCO_MOCOMINIMUMDISTANCECONSTRAINT_H