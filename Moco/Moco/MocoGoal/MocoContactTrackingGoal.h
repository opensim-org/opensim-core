#ifndef MOCO_MOCOCONTACTTRACKINGGOAL_H
#define MOCO_MOCOCONTACTTRACKINGGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoContactTrackingGoal.h                                    *
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
#include "../Components/SmoothSphereHalfSpaceForce.h"

namespace OpenSim {

/// A contact group is a single ExternalForce and a list of contact force
/// components in the model whose forces are summed and compared to the
/// ExternalForce.
/// @see MocoContactTrackingGoal
class OSIMMOCO_API MocoContactTrackingGoalGroup : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoContactTrackingGoalGroup, Object);
public:
    OpenSim_DECLARE_LIST_PROPERTY(contact_force_paths, std::string,
            "Paths to SmoothSphereHalfSpaceForce objects in the model whose "
            "forces are summed and compared to an single ExternalForce.");
    OpenSim_DECLARE_PROPERTY(external_force_name, std::string,
            "The name of an ExternalForce object in the ExternalLoads set.");
    MocoContactTrackingGoalGroup();
    MocoContactTrackingGoalGroup(
            const std::vector<std::string>& contactForcePaths,
            const std::string& externalForceName);
private:
    void constructProperties();
};


/// Minimize the error between compliant contact force elements in the model and
/// experimentally measured contact forces.
///
/// This class handles multiple groups of contact forces and a single
/// experimental external loads file. Tracking ground reaction forces for the
/// left and right feet in gait requires only one instance of this goal.
///
/// @note The only contact element supported is SmoothSphereHalfSpaceForce.
///
/// @note This goal does not include torques or centers of pressure.
///
/// This goal is computed as follows:
///
/// \f[
/// \frac{1}{mg} \int_{t_i}^{t_f}
///         \sum_{g \in G} \|\mathrm{proj}_{\hat{n}}(F_{m,g} - F_{e,g})\|^2 ~dt
/// \f]
/// We use the following notation:
/// - \f$ mg \f$: the total weight of the system.
/// - \f$ G \f$: the set of contact groups.
/// - \f$ \hat{n} \f$: a vector used for projecting the force error.
/// - \f$ \mathrm{proj}_{\hat{n}}() \f$: this function projects the force error
///     either onto \f$ \hat{n} \f$ or onto the plane perpendicular to
///     \f$ \hat{n} \f$.
/// - \f$ F_{m,g} \f$ the sum of the contact forces in group \f$ g \f$,
///     expressed in ground.
/// - \f$ F_{e,g} \f$ the experimental contact force for group \f$ g \f$,
///     expressed in ground.
///
/// If the model's gravity $g$ is 0, then we normalize by the total mass of the
/// system instead of the total weight.
///
/// ### Tracking a subset of force components
///
/// The projection is useful for selecting which components of the force to
/// track. The force can be projected to be onto a vector or
/// onto a plane. For example, with gait, projecting onto the vector (0, 1, 0)
/// allows tracking only the vertical component of a ground reaction force;
/// projecting onto the plane perpendicular to the vector (0, 0, 1) allows
/// ignoring the transverse force. See the projection and projection_vector
/// properties.
///
/// ### Usage
///
/// To use this goal, specify the following:
/// - a single ExternalLoads file or object, which is a set of ExternalForces.
/// - a set of contact groups, each of which contains the name of an
///     ExternalForce (within the ExternalLoads).
///
/// ### Configuring the ExternalLoads
///
/// The ExternalLoads class is the standard way to provide experimental contact
/// forces in OpenSim. This class is a set of ExternalForce objects. For gait,
/// typically the ExternalLoads contains 2 ExternalForces, one for each foot.
/// This goal uses the following information from ExternalLoads:
/// - **data_file**: This scalar file contains all experimental force data with
///   columns named according to each ExternalForce's force_identifier.
///
/// This goal uses the following information from each ExternalForce:
/// - **name**: We use the name of the ExternalForce to associate it with a
///   contact force group.
/// - **applied_on_body**: All contact forces in the group with which this
///   ExternalForce is associated must use this body as either the
///   sphere_frame's base frame or the half_space_frame's base frame.
/// - **force_expressed_in_body**: We use this to re-express the experimental
///   force in ground. This is either the absolute path to a PhysicalFrame in
///   the model, or the name of a Body in the model's BodySet.
/// - **force_identifier**: The ExternalLoads data_file must include the 3
///   columns
///   `<force_identifier>x`, `<force_identifier>y`, `<force_identifier>z`.
///
/// All other properties of ExternalLoads and ExternalForce are ignored by this
/// goal. This means that experimental forces are processed differently by this
/// goal than by other OpenSim tools such as Inverse Dynamics, Computed Muscle
/// Control, and Forward.
///
/// @note The ExternalLoads used by this goal is separate from the model. Using
/// this goal implies that the model contains compliant contact forces, so
/// adding ExternalLoads to the model would be redundant. This class uses the
/// ExternalLoads *only* for computing the force error, not for applying forces
/// to the model.
///
/// @ingroup mocogoal
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

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const SimTK::State& state, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.integral / m_denominator;
    }
    void printDescriptionImpl(std::ostream& stream = std::cout) const override;

private:
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

    void constructProperties();

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
    };
    mutable std::vector<GroupInfo> m_groups;
};

} // namespace OpenSim

#endif // MOCO_MOCOCONTACTTRACKINGGOAL_H
