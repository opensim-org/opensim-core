#ifndef OPENSIM_MOCOCONTACTTRACKINGGOAL_H
#define OPENSIM_MOCOCONTACTTRACKINGGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoContactTrackingGoal.h                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
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
#include <OpenSim/Simulation/Model/ExternalLoads.h>

namespace OpenSim {

class SmoothSphereHalfSpaceForce;
/** 
\section MocoContactTrackingGoalGroup
A contact group consists of the name of a single ExternalForce and a list of
contact force component paths in the model. The MocoContactTrackingGoal
calculates the difference between the data from the ExternalForce and the sum of
the forces from the contact force components.

## Alternative frame paths

Contact force elements that correspond to a single ExternalForce are
typically attached to the same single body/frame. However, it is possible
that these contact force elements are spread over multiple bodies. For
example, the ground reaction force for the left foot may be modeled by
contact force elements on separate calcaneus and toe body segments.
The "applied_to_body" property of the associated ExternalForce will match
only one of these multiple bodies (e.g., only the calcaneus), causing Moco
to give an error, saying one of the contact elements does not seem to be
associated with the ExternalForce. To handle this situation, specify the
*other* body (e.g., the toes) under alternative_frame_paths. Without
specifying these alternative frames, Moco does not know which force to use
(the force on the sphere or the force on the half-space) when summing the
contact forces across contact force elements.

@see MocoContactTrackingGoal */
class OSIMMOCO_API MocoContactTrackingGoalGroup : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoContactTrackingGoalGroup, Object);
public:
    OpenSim_DECLARE_LIST_PROPERTY(contact_force_paths, std::string,
            "Paths to SmoothSphereHalfSpaceForce objects in the model whose "
            "forces are summed and compared to the data from a single "
            "ExternalForce.");
    OpenSim_DECLARE_PROPERTY(external_force_name, std::string,
            "The name of an ExternalForce object in the ExternalLoads set.");
    OpenSim_DECLARE_LIST_PROPERTY(alternative_frame_paths, std::string,
            "If neither of the two bodies/frames of a contact force match "
            "ExternalForce's applied_to_body, then one of the bodies/frames "
            "must match one of these alternative frame paths.");
    MocoContactTrackingGoalGroup();
    MocoContactTrackingGoalGroup(
            const std::vector<std::string>& contactForcePaths,
            const std::string& externalForceName);
    MocoContactTrackingGoalGroup(
            const std::vector<std::string>& contactForcePaths,
            const std::string& externalForceName,
            const std::vector<std::string>& altFramePaths);
private:
    void constructProperties();
};


/**
\section MocoContactTrackingGoal
Minimize the error between compliant contact force elements in the model and
experimentally measured contact forces.

This class handles multiple groups of contact forces and a single
experimental external loads file. Tracking ground reaction forces for the
left and right feet in gait requires only one instance of this goal.

@note The only contact element supported is SmoothSphereHalfSpaceForce.

@note This goal does not include torques or centers of pressure.

This goal is computed as follows:

\f[
\frac{1}{mg} \int_{t_i}^{t_f}
        \sum_{j \in G}
            \|\mathrm{proj}_{\hat{n}}(\vec{F}_{m,j} - \vec{F}_{e,j})\|^2 ~dt
\f]
We use the following notation:
- \f$ t_i \f$: the initial time of this phase.
- \f$ t_f \f$: the final time of this phase.
- \f$ mg \f$: the total weight of the system; replaced with
    \f$ m \f$ if \f$ g = 0 \f$.
- \f$ G \f$: the set of contact groups.
- \f$ \hat{n} \f$: a vector used for projecting the force error.
- \f$ \mathrm{proj}_{\hat{n}}() \f$: this function projects the force error
    either onto \f$ \hat{n} \f$ or onto the plane perpendicular to
    \f$ \hat{n} \f$.
- \f$ \vec{F}_{m,j} \f$ the sum of the contact forces in group \f$ j \f$,
    expressed in ground.
- \f$ \vec{F}_{e,j} \f$ the experimental contact force for group \f$ j \f$,
    expressed in ground.

# Tracking a subset of force components

The projection is useful for selecting which components of the force to
track. The force can be projected to be onto a vector or
onto a plane. For example, with gait, projecting onto the vector (0, 1, 0)
allows tracking only the vertical component of a ground reaction force;
projecting onto the plane perpendicular to the vector (0, 0, 1) allows
ignoring the transverse force. See the projection and projection_vector
properties.

## Usage

To use this goal, specify the following:
- a single ExternalLoads file or object, which is a set of ExternalForces.
- a set of contact groups, each of which contains the name of an
    ExternalForce (within the ExternalLoads).

### Configuring the ExternalLoads

The ExternalLoads class is the standard way to provide experimental contact
forces in OpenSim. This class is a set of ExternalForce objects. For gait,
typically the ExternalLoads contains 2 ExternalForces, one for each foot.
This goal uses the following information from ExternalLoads:
- **data_file**: This scalar file contains all experimental force data with
  columns named according to each ExternalForce's force_identifier.

This goal uses the following information from each ExternalForce:
- **name**: We use the name of the ExternalForce to associate it with a
  contact force group.
- **applied_on_body**: All contact forces in the group with which this
  ExternalForce is associated must use this body as either the
  sphere_frame's base frame or the half_space_frame's base frame.
- **force_expressed_in_body**: We use this to re-express the experimental
  force in ground. This is either the absolute path to a PhysicalFrame in
  the model, or the name of a Body in the model's BodySet.
- **force_identifier**: The ExternalLoads data_file must include the 3
  columns
  `<force_identifier>x`, `<force_identifier>y`, `<force_identifier>z`.

All other properties of ExternalLoads and ExternalForce are ignored by this
goal. This means that experimental forces are processed differently by this
goal than by other OpenSim tools such as Inverse Dynamics, Computed Muscle
Control, and Forward.

@note The ExternalLoads used by this goal is separate from the model. Using
this goal implies that the model contains compliant contact forces, so
adding ExternalLoads to the model would be redundant. This class uses the
ExternalLoads *only* for computing the force error, not for applying forces
to the model.

### Scale factors

Add a MocoParameter to the problem that will scale the tracking reference
data associated with a contact force group. Scale factors are applied
to the tracking error calculations based on the following equation:

     error = modelValue - scaleFactor * referenceValue

In other words, the scale factor is applied when computing the tracking
error for each contact force group, not to the reference data directly.
You must specify both the external force name associated with the contact
force group and the index corresponding to the direction (i.e., X = 0,
Y = 1, Z = 2) of the scaled force value. The direction is applied in
whatever frame the reference data is expressed in based on the provided
ExternalLoads in each contact group.

Adding a scale factor to a MocoContactTrackingGoal.
@code
auto* contactTrackingGoal = problem.addGoal<MocoContactTrackingGoal>();
...
contactTrackingGoal->addScaleFactor(
        'RightGRF_vertical_scale_factor', 'Right_GRF', 1, {0.5, 2.0});
@endcode

@ingroup mocogoal */
class OSIMMOCO_API MocoContactTrackingGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoContactTrackingGoal, MocoGoal);

public:
    MocoContactTrackingGoal() { constructProperties(); }
    MocoContactTrackingGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoContactTrackingGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    /// Set the ExternalLoads as an XML file. This clears the ExternalLoads
    /// provided as an object, if one exists.
    void setExternalLoadsFile(const std::string& extLoadsFile);
    /// Set the ExternalLoads as an object. This clears the ExternalLoads
    /// XML file, if provided.
    void setExternalLoads(const ExternalLoads& extLoads);

    /// Add a group of contact forces whose sum should track the force data from
    /// a single ExternalForce. The externalForceName should be the name of an
    /// ExternalForce object in the ExternalLoads.
    void addContactGroup(
            const std::vector<std::string>& contactForcePaths,
            const std::string& externalForceName) {
        append_contact_groups(MocoContactTrackingGoalGroup(
                contactForcePaths, externalForceName));
    }
    /// Add a group of contact forces whose sum should track the force data from
    /// a single ExternalForce.
    /// If the contact force elements associated with a single ExternalForce are
    /// distributed across multiple bodies use this function instead of the
    /// easier-to-use addContactGroup(), and set the group's
    /// alternative_frame_paths property accordingly. See
    /// MocoContactTrackingGoalGroup for more information.
    void addContactGroup(MocoContactTrackingGoalGroup group) {
        append_contact_groups(std::move(group));
    }

    /// Set if the force error should be projected onto either a vector or
    /// plane. Possible values: "none" (default), "vector", and "plane".
    void setProjection(std::string projection) {
        set_projection(std::move(projection));
    }
    std::string getProjection() const { return get_projection(); }

    /// Set the vector to use for projecting the force error.
    /// If the projection type is "vector", the force error is projected onto
    /// the vector provided here. If the projection type is "plane", the force
    /// error is projected onto the plane perpendicular to this vector.
    void setProjectionVector(SimTK::Vec3 normal) {
        set_projection_vector(std::move(normal));
    }
    /// Unset the projection vector.
    void clearProjectionVector() { updProperty_projection_vector().clear(); }
    SimTK::Vec3 getProjectionVector() const { return get_projection_vector(); }

    /// Add a MocoParameter to the problem that will scale the tracking reference
    /// data associated with a contact force group. Scale factors are applied
    /// to the tracking error calculations based on the following equation:
    ///
    ///     error = modelValue - scaleFactor * referenceValue
    ///
    /// In other words, the scale factor is applied when computing the tracking
    /// error for each contact force group, not to the reference data directly.
    /// You must specify both the external force name associated with the contact
    /// force group and the index corresponding to the direction (i.e., X = 0,
    /// Y = 1, Z = 2) of the scaled force value. The direction is applied in
    /// whatever frame the reference data is expressed in based on the provided
    /// ExternalLoads in each contact group.
    void addScaleFactor(const std::string& name,
            const std::string& externalForceName, int index,
            const MocoBounds& bounds);

    /// Normalize each component of the 3-D tracking error by the peak value of 
    /// each contact force component in the tracking data. No normalization is 
    /// applied when tracking data is close to zero (default: false).
    void setNormalizeTrackingError(bool tf) {
        set_normalize_tracking_error(tf);
    }
    bool getNormalizeTrackingError() { return get_normalize_tracking_error(); }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.integral / m_denominator;
    }
    void printDescriptionImpl() const override;

private:
    // PROPERTIES
    OpenSim_DECLARE_LIST_PROPERTY(contact_groups, MocoContactTrackingGoalGroup,
            "Associate contact elements in the model with force data.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(external_loads, ExternalLoads,
            "Experimental contact force data.");
    OpenSim_DECLARE_PROPERTY(external_loads_file, std::string,
            "Experimental contact force data as an ExternalLoads XML file.");
    OpenSim_DECLARE_PROPERTY(projection, std::string,
            "'none' (default): use full 3-D force error; "
            "'vector': project force error onto projection_vector; "
            "'plane': project force error onto the plane perpendicular "
            "to projection_vector.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(projection_vector, SimTK::Vec3,
            "(optional) If provided, the force error is projected onto the "
            "plane perpendicular to this vector. The vector is expressed in "
            "ground. The vector can have any length.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(normalize_tracking_error, bool,
            "Normalize each component of the 3-D tracking error by the peak "
            "magnitude of each contact force component in the tracking data. "
            "If the peak magnitude of the ground contact force data is close "
            "to zero, an exception is thrown (default: false).");

    void constructProperties();

    /// For a given contact force, find the starting index of the forces from
    /// SmoothSphereHalfSpaceForce::getRecordValues().
    int findRecordOffset(
            const MocoContactTrackingGoalGroup& group,
            const SmoothSphereHalfSpaceForce& contactForce,
            const std::string& appliedToBody) const;

    enum class ProjectionType {
        None,
        Vector,
        Plane
    };
    mutable ProjectionType m_projectionType = ProjectionType::None;
    mutable SimTK::UnitVec3 m_projectionVector;
    mutable double m_denominator;

    /// Each contact group includes a list of contact force components (with an
    /// int that keeps track of whether we want to use the force applied to the
    /// sphere or to the half space) and a spline representation of associated
    /// experimental data.
    struct GroupInfo {
        std::vector<std::pair<const SmoothSphereHalfSpaceForce*, int>> contacts;
        GCVSplineSet refSplines;
        const PhysicalFrame* refExpressedInFrame = nullptr;
        SimTK::Vec3 normalizeFactors = SimTK::Vec3(1.0);
    };
    mutable std::vector<GroupInfo> m_groups;

    mutable std::map<std::pair<std::string, int>, std::string> m_scaleFactorMap;
    using RefPtrMSF = SimTK::ReferencePtr<const MocoScaleFactor>;
    mutable std::vector<std::array<RefPtrMSF, 3>> m_scaleFactorRefs;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOCONTACTTRACKINGGOAL_H
