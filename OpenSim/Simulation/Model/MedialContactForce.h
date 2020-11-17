#ifndef MEDIALCONTACTFORCE_H
#define MEDIALCONTACTFORCE_H
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  MedialContactForce.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include "OpenSim/Simulation/SimbodyEngine/Joint.h"

namespace OpenSim {

/**
A model component for conveniently computing medial contact force.
<b>Default %Property Values</b>
@verbatim

@endverbatim
 */
class OSIMSIMULATION_API MedialContactForce : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(MedialContactForce, ModelComponent);

public:
    OpenSim_DECLARE_PROPERTY(loads_frame, std::string,
            "TODO");
    OpenSim_DECLARE_PROPERTY(condyle_width, double,
            "default: 0.05 m.");
    OpenSim_DECLARE_PROPERTY(adduction_moment_index, int,
            "TODO");
    OpenSim_DECLARE_PROPERTY(vertical_force_index, int,
            "TODO");

    OpenSim_DECLARE_SOCKET(joint, Joint,
            "TODO.");

    OpenSim_DECLARE_OUTPUT(medial_contact_force, double, getMedialContactForce,
            SimTK::Stage::Dynamics);

    MedialContactForce();

    double getMedialContactForce(const SimTK::State& s) const;

private:
    void constructProperties();
    void extendFinalizeFromProperties() override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    void calcMedialContactForce(const SimTK::State& s, SimTK::Real& force);

    mutable SimTK::ReferencePtr<const Joint> m_joint;
    mutable bool m_isParentFrame;

};

} // namespace OpenSim

#endif // MEDIALCONTACTFORCE_H