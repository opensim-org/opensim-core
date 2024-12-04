#ifndef OPENSIM_MOCOCONTACTINITIALIZER_H
#define OPENSIM_MOCOCONTACTINITIALIZER_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoContactInitializer.h                                          *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 Stanford University and the Authors                     *
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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Actuators/ModelProcessor.h>
#include <OpenSim/Simulation/Model/ExternalLoads.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Moco/osimMocoDLL.h>

// TODO start with SmoothSphereHalfSpaceForce, then support other contact forces.

namespace OpenSim {

class OSIMMOCO_API MocoStaticContactSolver : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoStaticContactSolver, Object);

};


class OSIMMOCO_API MocoContactInitializerGroup : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoContactInitializerGroup, Object);
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
    MocoContactInitializerGroup();
    MocoContactInitializerGroup(
            const std::vector<std::string>& contactForcePaths,
            const std::string& externalForceName);
    MocoContactInitializerGroup(
            const std::vector<std::string>& contactForcePaths,
            const std::string& externalForceName,
            const std::vector<std::string>& altFramePaths);
private:
    void constructProperties();
};

class OSIMMOCO_API MocoContactInitializer : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoContactInitializer, Object);

public:
    MocoContactInitializer();

    /** TODO */
    void setModel(Model model);
    void setModelProcessor(ModelProcessor modelProcessor);
    const ModelProcessor& getModelProcessor() const { return get_model(); }


    /** Set the ExternalLoads as an XML file. This clears the ExternalLoads
    provided as an object, if one exists. */
    void setExternalLoadsFile(const std::string& extLoadsFile);

    /** Set the ExternalLoads as an object. This clears the ExternalLoads
    XML file, if provided. */
    void setExternalLoads(const ExternalLoads& extLoads);

    /** Add a group of contact forces whose sum should track the force data from
    a single ExternalForce. The externalForceName should be the name of an
    ExternalForce object in the ExternalLoads. */
    void addContactGroup(
            const std::vector<std::string>& contactForcePaths,
            const std::string& externalForceName) {
        append_contact_groups(MocoContactInitializerGroup(
                contactForcePaths, externalForceName));
    }

    /** Add a group of contact forces whose sum should track the force data from
    a single ExternalForce.
    If the contact force elements associated with a single ExternalForce are
    distributed across multiple bodies use this function instead of the
    easier-to-use addContactGroup(), and set the group's
    alternative_frame_paths property accordingly. See
    MocoContactInitializerGroup for more information. */
    void addContactGroup(MocoContactInitializerGroup group) {
        append_contact_groups(std::move(group));
    }

    /** Provide a MarkersReference object containing the marker trajectories to
    be tracked by a model. The MarkersReferences can be created from a file
    of marker trajectories (e.g. .trc) or created programmatically via a
    TimeSeriesTableVec3. The marker weights property can be optionally
    specified to weight the tracking of individual markers in the cost
    function. Names of markers in the reference to be tracked should match
    the names of corresponding model markers. */
    void setMarkersReference(const MarkersReference&);
    // TODO static MarkersReference createMarkersReference();
    // TODO static MarkersReference createMarkersReferenceFromCoordinates();

    // Model createContactBodyModel();

protected:
    OpenSim_DECLARE_PROPERTY(model, ModelProcessor, "TODO");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(external_loads, ExternalLoads,
            "Experimental contact force data.");
    OpenSim_DECLARE_PROPERTY(external_loads_file, std::string,
            "Experimental contact force data as an ExternalLoads XML file.");
    OpenSim_DECLARE_LIST_PROPERTY(contact_groups, MocoContactInitializerGroup,
            "Associate contact elements in the model with force data.");
    OpenSim_DECLARE_PROPERTY(markers_reference, MarkersReference, "TODO");

private:
    void constructProperties();

    Model initializeModel();

    void initializeExternalLoads(const Model& model);

    /// For a given contact force, find the starting index of the forces from
    /// SmoothSphereHalfSpaceForce::getRecordValues().
    // int findRecordOffset(
    //         const MocoContactInitializerGroup& group,
    //         const SmoothSphereHalfSpaceForce& contactForce,
    //         const std::string& appliedToBody) const;

    /// Each contact group includes a list of contact force components (with an
    /// int that keeps track of whether we want to use the force applied to the
    /// sphere or to the half space) and a spline representation of associated
    /// experimental data.
    struct GroupInfo {
        std::vector<std::pair<const Force*, int>> contacts;
        GCVSplineSet refSplines;
        const PhysicalFrame* refExpressedInFrame = nullptr;
        SimTK::Vec3 scaleFactors = SimTK::Vec3(1.0);
    };
    mutable std::vector<GroupInfo> m_groups;

};

} // namespace OpenSim

#endif // OPENSIM_MOCOCONTACTINITIALIZER_H
