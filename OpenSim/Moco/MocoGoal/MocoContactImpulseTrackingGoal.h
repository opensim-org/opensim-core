#ifndef OPENSIM_MOCOCONTACTIMPULSETRACKINGGOAL_H
#define OPENSIM_MOCOCONTACTIMPULSETRACKINGGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoContactImpulseTrackingGoal.h                                  *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2022 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicos Haralabidis                                               *
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

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/Model/ExternalLoads.h>

namespace OpenSim {

    class SmoothSphereHalfSpaceForce;
    /**
    \section MocoContactImpulseTrackingGoalGroup
    The MocoContactImpulseTrackingGoalGroup reflects the name of a single ExternalForce 
    and a list of smooth compliant contact force components in the model. 
    The MocoContactImpulseTrackingGoal calculates the difference in contact impulse, 
    for a single axis, between the data from the ExternalForce and the sum of the 
    forces from the contact force components.
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
    @see MocoContactImpulseTrackingGoal */
    class OSIMMOCO_API MocoContactImpulseTrackingGoalGroup : public Object {
        OpenSim_DECLARE_CONCRETE_OBJECT(MocoContactImpulseTrackingGoalGroup, Object);
    public:
        OpenSim_DECLARE_LIST_PROPERTY(contact_force_paths, std::string,
            "Paths to SmoothSphereHalfSpaceForce objects in the model whose "
            "combined contact impulse is compared to the contact impulse "
            "from a single ExternalForce.");
        OpenSim_DECLARE_PROPERTY(external_force_name, std::string,
            "The name of an ExternalForce object in the ExternalLoads set.");
        OpenSim_DECLARE_LIST_PROPERTY(alternative_frame_paths, std::string,
            "If neither of the two bodies/frames of a contact force match "
            "ExternalForce's applied_to_body, then one of the bodies/frames "
            "must match one of these alternative frame paths.");
        MocoContactImpulseTrackingGoalGroup();
        MocoContactImpulseTrackingGoalGroup(
            const std::vector<std::string>& contactForcePaths,
            const std::string& externalForceName);
        MocoContactImpulseTrackingGoalGroup(
            const std::vector<std::string>& contactForcePaths,
            const std::string& externalForceName,
            const std::vector<std::string>& altFramePaths);
    private:
        void constructProperties();
    };


    /**
    \section MocoContactImpulseTrackingGoal
    Minimize the error between compliant contact force impulse from the model and
    experimentally measured contact impulse for a single axis.
    This class handles the contact impulses from a single contact group and a 
    single experimental external loads file. Tracking ground reaction impulses for
    the left and right feet in gait requires separate instances of this goal.
    Tracking ground reaction impulses for multiple axes requires separate 
    instances of this goal.
    @note The only contact element supported is SmoothSphereHalfSpaceForce.
    @note This goal does not include torques or centers of pressure.
    This goal is computed as follows:
    \f[
    \frac{1}{mg} (\int_{t_i}^{t_f}
                 \vec{F}_{m,a} - \vec{F}_{e,a} ~dt)^2
    \f]
    We use the following notation:
    - \f$ t_i \f$: the initial time of this phase.
    - \f$ t_f \f$: the final time of this phase.
    - \f$ mg \f$: the total weight of the system; replaced with
        \f$ m \f$ if \f$ g = 0 \f$.
    - \f$ a \f$: the impulse axis to be tracked.

    The impulse_axis property, a = {0, 1, 2}, specifies which component
    of the contact impulse should be tracked.

    ## Usage
    To use this goal, specify the following:
    - a single ExternalLoads file or object, which is a set of ExternalForces.
    - a single contact group, which contains the names of an contact forces. 
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
    ExternalLoads *only* for computing the contact impulse error, not for 
    applying forces to the model.
    ### Scale factors
    Add a MocoParameter to the problem that will scale the tracking reference
    data associated with a contact force group. Scale factors are applied
    to the tracking error calculations based on the following equation:
         error = modelValue - scaleFactor * referenceValue
    In other words, the scale factor is applied when computing the contact
    impulse tracking error for each contact force group, not to the reference 
    data directly. You must specify both the external force name associated 
    with the contact force group and the index corresponding to the direction
    (i.e., X = 0, Y = 1, Z = 2) of the scaled force value. The direction is 
    applied in whatever frame the reference data is expressed in based on the 
    provided ExternalLoads in each contact group.
    Adding a scale factor to a MocoContactImpulseTrackingGoal.
    @code
    auto* contactImpulseTrackingGoal = problem.addGoal<MocoContactImpulseTrackingGoal>();
    ...
    contactImpulseTrackingGoal->addScaleFactor(
            'RightGRF_vertical_scale_factor', 'Right_GRF', 1, {0.5, 2.0});
    @endcode
    @ingroup mocogoal */
    class OSIMMOCO_API MocoContactImpulseTrackingGoal : public MocoGoal {
        OpenSim_DECLARE_CONCRETE_OBJECT(MocoContactImpulseTrackingGoal, MocoGoal);

    public:
        MocoContactImpulseTrackingGoal() { constructProperties(); }
        MocoContactImpulseTrackingGoal(std::string name) : MocoGoal(std::move(name)) {
            constructProperties();
        }
        MocoContactImpulseTrackingGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
            constructProperties();
        }

        /// Set the ExternalLoads as an XML file. This clears the ExternalLoads
        /// provided as an object, if one exists.
        void setExternalLoadsFile(const std::string& extLoadsFile);
        /// Set the ExternalLoads as an object. This clears the ExternalLoads
        /// XML file, if provided.
        void setExternalLoads(const ExternalLoads& extLoads);

        /// Add a single group of contact forces whose sum should track the ground 
        /// reaction force impulse data from a single axis of a single ExternalForce. 
        /// The externalForceName should be the name of an ExternalForce object
        /// in the ExternalLoads.
        void addContactGroup(
            const std::vector<std::string>& contactForcePaths,
            const std::string& externalForceName) {
            append_contact_groups(MocoContactImpulseTrackingGoalGroup(
                contactForcePaths, externalForceName));
        }
        /// Add a single group of contact forces whose sum should track the ground
        /// reaction force impulse data from a single axis of a single ExternalForce.
        /// If the contact force elements associated with a single ExternalForce are
        /// distributed across multiple bodies use this function instead of the
        /// easier-to-use addContactGroup(), and set the group's
        /// alternative_frame_paths property accordingly. See
        /// MocoContactImpulseTrackingGoalGroup for more information.
        void addContactGroup(MocoContactImpulseTrackingGoalGroup group) {
            append_contact_groups(std::move(group));
        }

        /// Set the ground reaction force contact impulse axis to be tracked
        /// (X = 0, Y = 1, Z = 2), where axis refers to the component of
        /// the ground reaction force contact impulse, in the ground frame, to be
        /// tracked.
        void setImpulseAxis(int impulseAxis) {
            set_impulse_axis(impulseAxis);
        }
        int getImpulseAxis() const { return get_impulse_axis(); }

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
            const std::string& externalForceName,
            const MocoBounds& bounds);

    protected:
        void initializeOnModelImpl(const Model&) const override;
        void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override;
        void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
            cost[0] = (input.integral * input.integral) / m_denominator;
        }
        void printDescriptionImpl() const override;

    private:
        // PROPERTIES
        OpenSim_DECLARE_LIST_PROPERTY(contact_groups, MocoContactImpulseTrackingGoalGroup,
            "Associate contact elements in the model with force data.");
        OpenSim_DECLARE_OPTIONAL_PROPERTY(external_loads, ExternalLoads,
            "Experimental contact force data.");
        OpenSim_DECLARE_PROPERTY(external_loads_file, std::string,
            "Experimental contact force data as an ExternalLoads XML file.");
        OpenSim_DECLARE_PROPERTY(impulse_axis, int,
            "The axis of the ground reaction force impulse component to be tracked in the goal."
            "X= 0, Y= 1 , Z= 2"
            "(Default value = -1)");

        void constructProperties();

        /// For a given contact force, find the starting index of the forces from
        /// SmoothSphereHalfSpaceForce::getRecordValues().
        int findRecordOffset(
            const MocoContactImpulseTrackingGoalGroup& group,
            const SmoothSphereHalfSpaceForce& contactForce,
            const std::string& appliedToBody) const;

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

        mutable std::map<std::string, std::string> m_scaleFactorMap;
        using RefPtrMSF = SimTK::ReferencePtr<const MocoScaleFactor>;
        mutable std::vector<RefPtrMSF> m_scaleFactorRefs;
    };

} // namespace OpenSim

#endif // OPENSIM_MOCOCONTACTIMPULSETRACKINGGOAL_H
