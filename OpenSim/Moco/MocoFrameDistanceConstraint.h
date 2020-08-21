#ifndef OPENSIM_MOCOFRAMEDISTANCECONSTRAINT_H
#define OPENSIM_MOCOFRAMEDISTANCECONSTRAINT_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoFrameDistanceConstraint.h                                     *
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

class OSIMMOCO_API MocoFrameDistanceConstraintPair : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoFrameDistanceConstraintPair, Object);

public:
    OpenSim_DECLARE_PROPERTY(frame1_path, std::string,
            "The first model frame path of the pair.");
    OpenSim_DECLARE_PROPERTY(frame2_path, std::string,
            "The second model frame path of the pair.");
    OpenSim_DECLARE_PROPERTY(minimum_distance, double,
            "The minimum distance apart that the two frame origins can be "
            "(meters).");
    OpenSim_DECLARE_PROPERTY(maximum_distance, double,
            "The maximum distance apart that the two frame origins can be "
            "(meters).")

    MocoFrameDistanceConstraintPair();
    MocoFrameDistanceConstraintPair(std::string firstFramePath,
            std::string secondFramePath, double minimum_distance,
            double maximum_distance);

private:
    void constructProperties();
};

/** This path constraint enforces that the distance between the origins of pairs
of model frames is kept between minimum and maximum bounds. Frame pairs and
their bounds are specified via a MocoFrameDistancConstraintPair.
Any model component derived from Frame is valid to be included in a frame
pair, and any number of frame pairs may be append to this constraint via
addFramePair().

This constraint can be used as a simple method for preventing bodies in your
model from intersecting during an optimization. For example, the
following prevents feet from intersecting during a walking optimization:
@code
distance = problem.addPathConstraint<MocoFrameDistanceConstraint>();
distance.setName("minimum_distance");
SimTK::Real inf = SimTK::Infinity;
distance.addFramePair('/bodyset/calcn_l', '/bodyset/calcn_r', 0.1, inf);
distance.addFramePair('/bodyset/toes_l', '/bodyset/toes_r', 0.1, inf);
distance.addFramePair('/bodyset/calcn_l', '/bodyset/toes_r', 0.1, inf);
distance.addFramePair('/bodyset/toes_l', '/bodyset/calcn_r', 0.1, inf);
@endcode

To project the frame distance onto a vector or plane before ensuring its
within the provided bounds, use setProjection() and setProjectionVector().

@note This class represents a path constraint, *not* a model kinematic
constraint. Therefore, there are no Lagrange multipliers or constraint
forces associated with this constraint. The model's force elements
(including actuators) must generate the forces necessary for satisfying this
constraint.

@ingroup mocopathcon */
class OSIMMOCO_API MocoFrameDistanceConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoFrameDistanceConstraint, MocoPathConstraint);

public:
    MocoFrameDistanceConstraint();

    void addFramePair(MocoFrameDistanceConstraintPair pair) {
        append_frame_pairs(std::move(pair));
    }
    void addFramePair(const std::string& frame1_path, 
            const std::string& frame2_path, double minimum_distance, 
            double maximum_distance) {
        append_frame_pairs(MocoFrameDistanceConstraintPair(frame1_path, 
            frame2_path, minimum_distance, maximum_distance));
    }

    /// Set if each distance should be projected onto either a vector or
    /// plane. Possible values: "none" (default), "vector", and "plane".
    void setProjection(std::string projection) {
        set_projection(std::move(projection));
    }
    std::string getProjection() const { return get_projection(); }

    /// Set the vector to use for projecting each distance.
    /// If the projection type is "vector", the distance is projected onto
    /// the vector provided here. If the projection type is "plane", the
    /// distance is projected onto the plane perpendicular to this vector.
    void setProjectionVector(SimTK::Vec3 vector) {
        set_projection_vector(std::move(vector));
    }
    /// Unset the projection vector.
    void clearProjectionVector() { updProperty_projection_vector().clear(); }
    SimTK::Vec3 getProjectionVector() const { return get_projection_vector(); }

protected:
    void initializeOnModelImpl(
            const Model& model, const MocoProblemInfo&) const override;
    void calcPathConstraintErrorsImpl(
            const SimTK::State& state, SimTK::Vector& errors) const override;

private:
    OpenSim_DECLARE_LIST_PROPERTY(frame_pairs, 
            MocoFrameDistanceConstraintPair, 
            "Pairs of frames whose origins are constrained to be within "
            "minimum and maximum bounds.");

    OpenSim_DECLARE_PROPERTY(projection, std::string,
            "'none' (default): use full 3-D distance; "
            "'vector': project distance onto projection_vector; "
            "'plane': project distance onto the plane perpendicular "
            "to projection_vector.");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(projection_vector, SimTK::Vec3,
            "(optional) If provided, the distance is projected onto the "
            "plane perpendicular to this vector. The vector is expressed in "
            "ground and can have any length.");

    void constructProperties();

    enum class ProjectionType {
        None,
        Vector,
        Plane
    };
    mutable ProjectionType m_projectionType = ProjectionType::None;
    mutable SimTK::UnitVec3 m_projectionVector;

    mutable std::vector<std::pair<SimTK::ReferencePtr<const Frame>,
            SimTK::ReferencePtr<const Frame>>> m_frame_pairs;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOFRAMEDISTANCECONSTRAINT_H
