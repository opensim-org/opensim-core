#ifndef OPENSIM_MOCOFRAMEORIENTATIONCONSTRAINT_H 
#define OPENSIM_MOCOFRAMEORIENTATIONCONSTRAINT_H 
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoFrameOrientationConstraint.h                             *
 * -------------------------------------------------------------------------- *
* Copyright (c) 2022 Stanford University and the Authors                      *
 *                                                                            *
 * Author(s): Size Zheng                                                      *
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
#include <OpenSim/Moco/osimMoco.h>
#include "osimMocoDLL.h"

namespace OpenSim {

class OSIMMOCO_API MocoFrameOrientationConstraintPair : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoFrameOrientationConstraintPair, Object);

public:
    OpenSim_DECLARE_PROPERTY(frame1_path, std::string,
        "The first model frame path of the pair.");
    OpenSim_DECLARE_PROPERTY(frame2_path, std::string,
        "The second model frame path of the pair.");
    OpenSim_DECLARE_PROPERTY(minimum_angle, double,
        "The minimum distance apart that the two frame origins can be "
        "(meters).");
    OpenSim_DECLARE_PROPERTY(maximum_angle, double,
        "The maximum distance apart that the two frame origins can be "
        "(meters).")

    MocoFrameOrientationConstraintPair();
    MocoFrameOrientationConstraintPair(std::string firstFramePath,
        std::string secondFramePath, double minimum_angle,
        double maximum_angle);

private:
    void constructProperties();
};

/** This path constraint enforces that the angle between the pairs of model 
frames is kept between minimum and maximum bounds. Frame pairs and
their bounds are specified via a MocoFrameOrientationConstraintPair.
Any model component derived from Frame is valid to be included in a frame
pair, and any number of frame pairs may be append to this constraint via
addFramePair().
To project the frame angle onto a vector or plane before ensuring its
within the provided bounds, use setProjection() and setProjectionVector().
@note This class represents a path constraint, *not* a model kinematic
constraint. Therefore, there are no Lagrange multipliers or constraint
forces associated with this constraint. The model's force elements
(including actuators) must generate the forces necessary for satisfying this
constraint.
@ingroup mocopathcon */
class OSIMMOCO_API MocoFrameOrientationConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoFrameOrientationConstraint, MocoPathConstraint);

public:
    MocoFrameOrientationConstraint();
    
    void addFramePair(MocoFrameOrientationConstraintPair pair) {
        append_frame_pairs(std::move(pair));
    }
    void addFramePair(const std::string& frame1_path,const std::string& frame2_path, double minimum_angle,double maximum_angle) {
        append_frame_pairs(MocoFrameOrientationConstraintPair(frame1_path,frame2_path, minimum_angle,maximum_angle));
    void setCoordinateAxis(int coordinate_axis) {
        set_coordinate_axis(coordinate_axis);
    }
    }
    
protected:
    void initializeOnModelImpl(const Model& model, const MocoProblemInfo&) const override;
    void calcPathConstraintErrorsImpl(const SimTK::State& state, SimTK::Vector& errors) const override;
    
private:
    OpenSim_DECLARE_LIST_PROPERTY(frame_pairs,
        MocoFrameOrientationConstraintPair,
        "Pairs of frames whose origins are constrained to be within "
        "minimum and maximum bounds.");
     OpenSim_DECLARE_OPTIONAL_PROPERTY(coordinate_axis, int,
        "(optional) If provided, the coordinate axis is defined "
        "0 represents x-aixs, 1 represents y-axis, 2 represents z-axis,"
        "3 represents all the axis.");
    
    void constructProperties();
    mutable std::vector<std::pair<SimTK::ReferencePtr<const Frame>,
        SimTK::ReferencePtr<const Frame>>> m_frame_pairs;
    mutable int m_coordinateAxis;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOFRAMEORIENTATIONCONSTRAINT_H 
